#include "jt_dynamic_controller/dynamic_compensation_node.h" 
#include "jt_admittance_msgs/msg/admittance_params.hpp"
#include <csignal> // LIBRERÍA OBLIGATORIA PARA EL MANEJO DE CTRL+C

using std::placeholders::_1;
using std::placeholders::_2;

// Puntero global para poder acceder al nodo desde el interceptor de señales
std::shared_ptr<dynamic_compensation_class> global_node_ptr = nullptr;

// Función que intercepta la señal de apagado (Ctrl+C)
void sigint_handler(int sig)
{
    (void)sig;
    if (global_node_ptr) {
        RCLCPP_INFO(global_node_ptr->get_logger(), "Ctrl+C / Apagado detectado. Deteniendo motores...");
        global_node_ptr->stop_robot(); // Detiene el robot MIENTRAS ROS 2 sigue vivo
    }
    rclcpp::shutdown(); // Ahora sí, apagamos ROS 2 limpiamente
}

int main(int argc, char ** argv)
{
    // Iniciamos ROS 2, pero bloqueamos el apagado automático por defecto
    rclcpp::InitOptions init_options;
    init_options.shutdown_on_signal = false;
    rclcpp::init(argc, argv, init_options);

    // Creamos el nodo asignándolo al puntero global
    global_node_ptr = std::make_shared<dynamic_compensation_class>();
    
    // Redirigimos la señal de Ctrl+C a nuestra función personalizada
    signal(SIGINT, sigint_handler);
    
    // rclcpp::spin bloquea el programa aquí y lo mantiene vivo
    rclcpp::spin(global_node_ptr->get_node_base_interface());

    return 0;
}

dynamic_compensation_class::dynamic_compensation_class()
: rclcpp::Node("dynamic_controller_node")
{
    this->declare_parameter<float>("mu", 20.41);
    this->declare_parameter<float>("mw", 3.27);
    this->declare_parameter<float>("du", 33.76);
    this->declare_parameter<float>("dw", 5.40);

    this->get_parameter("mu", mu);
    this->get_parameter("mw", mw);
    this->get_parameter("du", du);
    this->get_parameter("dw", dw);

    this->declare_parameter<float>("user/weight", 60.0);
    this->get_parameter("user/weight", user_weight);
    min_force_z = user_weight * 0.05;
    
    // Inicialización de variables matemáticas
    ts_ = 0.01; 
    u_ = 0.0; w_ = 0.0;

    default_chi_.resize(6);
    default_chi_ << 0.2421, 0.2792, -0.0013, 1.0001, 0.2735, 0.9529;
    
    adaptive_chi_.resize(6);
    adaptive_chi_ = default_chi_; // Inicializamos con los valores seguros por defecto

    chi_.resize(6);
    chi_ = default_chi_; // Iniciamos usando los parámetros por defecto

    M_ << chi_(0), 0.0,
          0.0, chi_(1);

    Fx_filt_ = 0.0; Fx_p_filt_ = 0.0; Fx_res_filtr_ = 0.0;
    Tz_filt_ = 0.0; Tz_p_filt_ = 0.0; Tz_res_filtr_ = 0.0;

    alpha_Fx_ = 1.0; 
    beta_Fx_ = pow(alpha_Fx_, 2) / (2 - alpha_Fx_);
    alpha_Tz_ = 1.0; 
    beta_Tz_ = pow(alpha_Tz_, 2) / (2 - alpha_Tz_);

// 1. Declaramos los límites de esfuerzo (saturaciones) como parámetros dinámicos
    this->declare_parameter<double>("Ku1", 0.6); // Valor sugerido inicial
    this->declare_parameter<double>("Kw1", 1.47);

    // 2. Declaramos los factores de sensibilidad (lambdas)
    this->declare_parameter<double>("lambda_u", 0.5);
    this->declare_parameter<double>("lambda_w", 2.0);

    // 3. Obtenemos los valores iniciales
    this->get_parameter("Ku1", Ku1);
    this->get_parameter("Kw1", Kw1);
    this->get_parameter("lambda_u", lambda_u);
    this->get_parameter("lambda_w", lambda_w);

    // 4. Calculamos Ku2 y Kw2 internamente 
    Ku2 = lambda_u / Ku1;
    Kw2 = lambda_w / Kw1;

    // Publisher
    dinamic_control_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/dynamic_control/cmd_vel", 10);

    tuning_params_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/dynamic_control/tuning_params", 10);

    admittance_params_sub_ = this->create_subscription<jt_admittance_msgs::msg::AdmittanceParams>(
      "admittance_modulator/admittance_params", 100, std::bind(&dynamic_compensation_class::admittance_params_Callback, this, _1));

    theta_Sub = create_subscription<std_msgs::msg::Float64MultiArray>(
      "adaptive_params", 10, std::bind(&dynamic_compensation_class::theta_Callback, this, _1));

    virtual_wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "obstacle_avoidance/virtual_wrench", 10, std::bind(&dynamic_compensation_class::virtual_wrench_Callback, this, _1));

    kin_ref_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      "admittance_control/cmd_vel", 10, std::bind(&dynamic_compensation_class::kin_ref_Callback, this, _1));

    // Configuración de los suscriptores para la sincronización
    sub_vel_filtered_.subscribe(this, "/odom_filtered");
    //sub_vel_filtered_.subscribe(this, "/odom");
    sub_force_.subscribe(this, "force/decomposed"); 

    // Configuración del sincronizador
    sync_sub_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), sub_vel_filtered_, sub_force_);
    sync_sub_->registerCallback(std::bind(&dynamic_compensation_class::callback, this, _1, _2));

    parameter_subscription_ = this->add_on_set_parameters_callback(std::bind(&dynamic_compensation_class::parametersCallback, this, _1));

    force_raw_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "force/raw", 100, std::bind(&dynamic_compensation_class::force_raw_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Dynamic compensation node started.");
}

dynamic_compensation_class::~dynamic_compensation_class()
{
    RCLCPP_INFO(this->get_logger(), "Shutting down");
}

void dynamic_compensation_class::stop_robot() 
{
    RCLCPP_INFO(this->get_logger(), "Ctrl+C detected / Node shutting down! Sending final message...");
    
    auto msg = geometry_msgs::msg::TwistStamped();
    msg.header.stamp = this->now(); // Usamos el tiempo actual del reloj de ROS
    msg.header.frame_id = "base_link";
    msg.twist.linear.x = 0.0;
    msg.twist.angular.z = 0.0;
    
    dinamic_control_pub_->publish(msg);
    dinamic_control_pub_->publish(msg);
    dinamic_control_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Node exited gracefully.");
}

void dynamic_compensation_class::callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg1, 
                                          const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& msg2)
{
    // Extraemos valores
    u_ = msg1->twist.twist.linear.x;  
    w_ = msg1->twist.twist.angular.z;

    Fx_ = msg2->wrench.force.x*9.8;
    //Tz_ = -msg2->wrench.torque.z*9.8;
    Tz_ = -(msg2->wrench.torque.z + virtual_torque_z_) * 9.8;

    double threshold_low = 0.25 * min_force_z;
    double threshold_high = 0.5 * min_force_z;

    if (force_left_z > threshold_low && force_right_z > threshold_low) {
        is_active_ = true;
    } else if (force_left_z < threshold_high || force_right_z < threshold_high) {
        is_active_ = false;
    }

    double u_d = 0.0;
    double w_d = 0.0;

    // --- 2. Cálculo de Compensación Dinámica ---
    if (!is_active_) {
        // Si no hay usuario, limpiar la "memoria" de los filtros Alfa-Beta
        Fx_filt_ = 0.0; 
        Fx_p_filt_ = 0.0; 
        Fx_res_filtr_ = 0.0;

        Tz_filt_ = 0.0; 
        Tz_p_filt_ = 0.0; 
        Tz_res_filtr_ = 0.0;
        
        // Las salidas u_d y w_d ya son 0.0
    } else {
        // 1. Vector de velocidad REAL (de la odometría) para la matemática original
        Eigen::Vector2d V(u_, w_);

        // 2. Matrices H (Inercia) y C (Coriolis/Fricción) Originales
        Eigen::Matrix2d H;
        H << chi_(0), 0.0,
             0.0,     chi_(1); 

        Eigen::Matrix2d C;
        C << chi_(3),       -chi_(2) * w_,
             chi_(4) * w_,   chi_(5);

        // Filtro aceleración (Mantenido igual)
        Fx_filt_ = Fx_filt_ + (Fx_p_filt_ * ts_);
        Fx_res_filtr_ = Fx_ - Fx_filt_;
        Fx_filt_ = Fx_filt_ + alpha_Fx_ * Fx_res_filtr_;
        Fx_p_filt_ = Fx_p_filt_ + (beta_Fx_ * Fx_res_filtr_) / ts_;

        Tz_filt_ = Tz_filt_ + (Tz_p_filt_ * ts_);
        Tz_res_filtr_ = Tz_ - Tz_filt_;
        Tz_filt_ = Tz_filt_ + alpha_Tz_ * Tz_res_filtr_;
        Tz_p_filt_ = Tz_p_filt_ + (beta_Tz_ * Tz_res_filtr_) / ts_;

        // 3. Cálculo de aceleraciones deseadas usando referencias (uk_, wk_)
        uk_dot_ = (Fx_filt_ - du * uk_) / mu;
        wk_dot_ = (Tz_filt_ - dw * wk_) / mw;

        // 4. Término Sigma
        Eigen::Vector2d sigma(uk_dot_ + Ku1 * std::tanh(Ku2*(uk_-u_)), 
                              wk_dot_ + Kw1 * std::tanh(Kw2*(wk_-w_)));

        // 5. Ley de control final multiplicando por V REAL (El secreto del éxito)
        Eigen::Vector2d Vd = (H * sigma) + (C * V);

        // Saturaciones finales de hardware
        if (Vd(0) >= 0.6) Vd(0) = 0.6;
        else if (Vd(0) <= 0.0) Vd(0) = 0.0;

        if (Vd(1) >= 1.47) Vd(1) = 1.47;
        else if (Vd(1) <= -1.47) Vd(1) = -1.47;

        u_d = Vd(0);
        w_d = Vd(1);
    }

    // Publicar la velocidad de control
    auto msg_out = geometry_msgs::msg::TwistStamped();
    msg_out.header.stamp = msg1->header.stamp; 
    msg_out.header.frame_id = "base_link"; 
    msg_out.twist.linear.x = u_d;
    msg_out.twist.angular.z = w_d;
    dinamic_control_pub_->publish(msg_out);

    // NUEVO: Publicar los parámetros de sintonización para el rosbag
    auto params_msg = std_msgs::msg::Float64MultiArray();
    params_msg.data = {Ku1, Kw1, lambda_u, lambda_w};
    tuning_params_pub_->publish(params_msg);
}

void dynamic_compensation_class::admittance_params_Callback(const jt_admittance_msgs::msg::AdmittanceParams::SharedPtr msg) {
    // Extraemos todos los parámetros del mismo mensaje al instante
    mu = msg->m_u;
    du = msg->d_u;
    mw = msg->m_w;
    dw = msg->d_w;
}

void dynamic_compensation_class::kin_ref_Callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    // Actualizamos las velocidades cinemáticas de referencia con los datos del tópico
    uk_ = msg->twist.linear.x;
    wk_ = msg->twist.angular.z;
}

void dynamic_compensation_class::virtual_wrench_Callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg){
    virtual_torque_z_ = msg->wrench.torque.z;
}

void dynamic_compensation_class::theta_Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (msg->data.size() == 6) {
        for (int i = 0; i < 6; ++i) {
            chi_(i) = msg->data[i];
        }

        // Actualizamos la matriz de inercias M_ inmediatamente
        M_ << chi_(0), 0.0,
              0.0, chi_(1);
    }
}

void dynamic_compensation_class::force_raw_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr force_aux){
    force_left_z  = force_aux->wrench.force.z;
    force_right_z = force_aux->wrench.torque.z; 
}

rcl_interfaces::msg::SetParametersResult dynamic_compensation_class::parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
{
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;

    for (const auto &parameter : parameters) {
        if (parameter.get_name() == "mu") {
            mu = parameter.as_double();
            //RCLCPP_INFO(this->get_logger(), "Parámetro mv actualizado a: %f", mu);
        } 
        else if (parameter.get_name() == "mw") {
            mw = parameter.as_double();
            //RCLCPP_INFO(this->get_logger(), "Parámetro mw actualizado a: %f", mw);
        } 
        else if (parameter.get_name() == "du") {
            du = parameter.as_double();
            //RCLCPP_INFO(this->get_logger(), "Parámetro dv actualizado a: %f", du);
        } 
        else if (parameter.get_name() == "dw") {
            dw = parameter.as_double();
            //RCLCPP_INFO(this->get_logger(), "Parámetro dw actualizado a: %f", dw);
        }
        if (parameter.get_name() == "Ku1") {
            Ku1 = parameter.as_double();
            Ku2 = lambda_u / Ku1; // Recálculo instantáneo
            RCLCPP_INFO(this->get_logger(), "Nuevo Ku1: %f | Recalculado Ku2: %f", Ku1, Ku2);
        }
        else if (parameter.get_name() == "Kw1") {
            Kw1 = parameter.as_double();
            Kw2 = lambda_w / Kw1; // Recálculo instantáneo
            RCLCPP_INFO(this->get_logger(), "Nuevo Kw1: %f | Recalculado Kw2: %f", Kw1, Kw2);
        }
        else if (parameter.get_name() == "lambda_u") {
            lambda_u = parameter.as_double();
            Ku2 = lambda_u / Ku1; 
            RCLCPP_INFO(this->get_logger(), "Nuevo lambda_u: %f | Recalculado Ku2: %f", lambda_u, Ku2);
        } 
        else if (parameter.get_name() == "lambda_w") {
            lambda_w = parameter.as_double();
            Kw2 = lambda_w / Kw1; 
            RCLCPP_INFO(this->get_logger(), "Nuevo lambda_w: %f | Recalculado Kw2: %f", lambda_w, Kw2);
        }
    }

    return result;
}