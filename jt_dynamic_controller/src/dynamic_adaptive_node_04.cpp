#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int8.hpp> 
#include <Eigen/Dense>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "jt_admittance_msgs/msg/admittance_params.hpp"

class ParameterEstimatorNode : public rclcpp::Node {
public:
    ParameterEstimatorNode() : Node("parameter_estimator_node") {
        // Inicializar parámetros base de identificación (theta original)
        theta_ << 0.2421, 0.2792, -0.0013, 1.0001, 0.2735, 0.9529;
        theta_hat_ = theta_; // Inicializar estimación

        // --- DECLARACIÓN DE PARÁMETROS DINÁMICOS PARA MATRICES ---
        
        // Parámetros para gamma_inv_ (Ganancia de adaptación)
        this->declare_parameter<double>("gamma_inv_1", 0.2);
        this->declare_parameter<double>("gamma_inv_2", 0.6);
        this->declare_parameter<double>("gamma_inv_3", 0.1);
        this->declare_parameter<double>("gamma_inv_4", 0.1);
        this->declare_parameter<double>("gamma_inv_5", 0.8);
        this->declare_parameter<double>("gamma_inv_6", 0.5);

        // Parámetros para Gamma_ (Fuga / Leakage)
        this->declare_parameter<double>("Gamma_1", 0.0005);
        this->declare_parameter<double>("Gamma_2", 0.003);
        this->declare_parameter<double>("Gamma_3", 0.0001);
        this->declare_parameter<double>("Gamma_4", 0.0001);
        this->declare_parameter<double>("Gamma_5", 0.001);
        this->declare_parameter<double>("Gamma_6", 0.001);

        // Obtener valores iniciales y asignarlos a las matrices
        double gi1, gi2, gi3, gi4, gi5, gi6;
        this->get_parameter("gamma_inv_1", gi1);
        this->get_parameter("gamma_inv_2", gi2);
        this->get_parameter("gamma_inv_3", gi3);
        this->get_parameter("gamma_inv_4", gi4);
        this->get_parameter("gamma_inv_5", gi5);
        this->get_parameter("gamma_inv_6", gi6);
        gamma_inv_.diagonal() << gi1, gi2, gi3, gi4, gi5, gi6;

        double G1, G2, G3, G4, G5, G6;
        this->get_parameter("Gamma_1", G1);
        this->get_parameter("Gamma_2", G2);
        this->get_parameter("Gamma_3", G3);
        this->get_parameter("Gamma_4", G4);
        this->get_parameter("Gamma_5", G5);
        this->get_parameter("Gamma_6", G6);
        Gamma_.diagonal() << G1, G2, G3, G4, G5, G6;

        Ku1 = 0.6;
        Kw1 = 1.47;

        // Configuración del filtro Alfa-Beta (Idéntico al dinámico)
        alpha_Fx_ = 1.0; 
        beta_Fx_ = pow(alpha_Fx_, 2) / (2 - alpha_Fx_);
        alpha_Tz_ = 1.0; 
        beta_Tz_ = pow(alpha_Tz_, 2) / (2 - alpha_Tz_);

        // Declarar los factores de relajación (lambdas) como parámetros dinámicos
        this->declare_parameter<double>("lambda_u", 0.5);
        this->declare_parameter<double>("lambda_w", 2.0);

        // Obtener los valores iniciales
        this->get_parameter("lambda_u", lambda_u);
        this->get_parameter("lambda_w", lambda_w);

        // Calcular Ku2 y Kw2 iniciales
        Ku2 = lambda_u / Ku1;
        Kw2 = lambda_w / Kw1;

        // Registrar el callback para detectar cambios en los parámetros
        parameter_subscription_ = this->add_on_set_parameters_callback(
            std::bind(&ParameterEstimatorNode::parametersCallback, this, std::placeholders::_1));

        // Suscriptores
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                u_ = msg->twist.twist.linear.x;
                w_ = msg->twist.twist.angular.z;
                latest_odom_stamp_ = msg->header.stamp;
            });

        // Suscriptor de fuerzas descompuestas (Para Fx y Tz)
        sub_force_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "force/decomposed", 10, [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
                Fx_ = msg->wrench.force.x * 9.8;
                Tz_ = -msg->wrench.torque.z * 9.8;
            });

        // Suscriptor para la fuerza pura (raw) - Lógica de reinicio
        sub_force_raw_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "force/raw", 10, [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
                double force_left_z = msg->wrench.force.z;
                double force_right_z = msg->wrench.torque.z;
                this->Fz_ = force_left_z + force_right_z;
            });

        sub_admittance_params_ = this->create_subscription<jt_admittance_msgs::msg::AdmittanceParams>(
            "admittance_modulator/admittance_params", 10, 
            [this](const jt_admittance_msgs::msg::AdmittanceParams::SharedPtr msg) { 
                mu = msg->m_u; 
                du = msg->d_u; 
                mw = msg->m_w; 
                dw = msg->d_w; 
            });

        sub_adaptation_ = this->create_subscription<std_msgs::msg::Int8>(
            "/hmi_controller/dynamic_adaptation", 10,
            [this](const std_msgs::msg::Int8::SharedPtr msg) {
                if (this->select_adaptation_ == 0 && msg->data == 1) {
                    this->theta_hat_ = this->theta_; 
                    RCLCPP_INFO(this->get_logger(), "Adaptación ACTIVADA. Parámetros reiniciados a valores nominales.");
                } 
                else if (this->select_adaptation_ == 1 && msg->data == 0) {
                    this->theta_hat_ = this->theta_; 
                    RCLCPP_INFO(this->get_logger(), "Adaptación DESACTIVADA.");
                }
                this->select_adaptation_ = msg->data;
            });

        // Tópico de la referencia cinemática
        sub_kin_ref_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "admittance_control/cmd_vel", 10,
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                this->uk_ = msg->twist.linear.x;
                this->wk_ = msg->twist.angular.z;
            });
        
        // Publicador de los nuevos parámetros
        pub_params_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("adaptive_params", 10);

        // Bucle de control a 100Hz (Ts = 0.01)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ParameterEstimatorNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Dynamic adaptative node started.");
    }

    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;

        for (const auto &parameter : parameters) {
            // Lambdas
            if (parameter.get_name() == "lambda_u") {
                lambda_u = parameter.as_double();
                Ku2 = lambda_u / Ku1; 
                RCLCPP_INFO(this->get_logger(), "Nuevo lambda_u: %f", lambda_u);
            } 
            else if (parameter.get_name() == "lambda_w") {
                lambda_w = parameter.as_double();
                Kw2 = lambda_w / Kw1; 
                RCLCPP_INFO(this->get_logger(), "Nuevo lambda_w: %f", lambda_w);
            }
            // gamma_inv_
            else if (parameter.get_name() == "gamma_inv_1") {
                gamma_inv_.diagonal()[0] = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Nuevo gamma_inv_1: %f", gamma_inv_.diagonal()[0]);
            }
            else if (parameter.get_name() == "gamma_inv_2") {
                gamma_inv_.diagonal()[1] = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Nuevo gamma_inv_2: %f", gamma_inv_.diagonal()[1]);
            }
            else if (parameter.get_name() == "gamma_inv_3") {
                gamma_inv_.diagonal()[2] = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Nuevo gamma_inv_3: %f", gamma_inv_.diagonal()[2]);
            }
            else if (parameter.get_name() == "gamma_inv_4") {
                gamma_inv_.diagonal()[3] = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Nuevo gamma_inv_4: %f", gamma_inv_.diagonal()[3]);
            }
            else if (parameter.get_name() == "gamma_inv_5") {
                gamma_inv_.diagonal()[4] = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Nuevo gamma_inv_5: %f", gamma_inv_.diagonal()[4]);
            }
            else if (parameter.get_name() == "gamma_inv_6") {
                gamma_inv_.diagonal()[5] = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Nuevo gamma_inv_6: %f", gamma_inv_.diagonal()[5]);
            }
            // Gamma_
            else if (parameter.get_name() == "Gamma_1") {
                Gamma_.diagonal()[0] = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Nuevo Gamma_1: %f", Gamma_.diagonal()[0]);
            }
            else if (parameter.get_name() == "Gamma_2") {
                Gamma_.diagonal()[1] = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Nuevo Gamma_2: %f", Gamma_.diagonal()[1]);
            }
            else if (parameter.get_name() == "Gamma_3") {
                Gamma_.diagonal()[2] = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Nuevo Gamma_3: %f", Gamma_.diagonal()[2]);
            }
            else if (parameter.get_name() == "Gamma_4") {
                Gamma_.diagonal()[3] = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Nuevo Gamma_4: %f", Gamma_.diagonal()[3]);
            }
            else if (parameter.get_name() == "Gamma_5") {
                Gamma_.diagonal()[4] = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Nuevo Gamma_5: %f", Gamma_.diagonal()[4]);
            }
            else if (parameter.get_name() == "Gamma_6") {
                Gamma_.diagonal()[5] = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Nuevo Gamma_6: %f", Gamma_.diagonal()[5]);
            }
        }
        return result;
    }

private:
    void control_loop() {
        // Prevención de división por cero
        if (mu <= 0.001 || mw <= 0.001) return;

        // 1. Evaluar Lógica de Inactividad
        bool is_inactive = (std::abs(u_) < threshold_u_ && std::abs(w_) < threshold_w_ && std::abs(Fz_) < threshold_Fz_);

        if (is_inactive) {
            if (!was_reset_logged_) {
                RCLCPP_INFO(this->get_logger(), "Usuario desconectado y robot detenido. Parámetros congelados en base nominal.");
                was_reset_logged_ = true; 
            }
        } else {
            was_reset_logged_ = false; 
        }

        // 2. Filtro Alfa-Beta para la fuerza (Siguen corriendo para mantener estabilidad)
        Fx_filt_ = Fx_filt_ + (Fx_p_filt_ * Ts_);
        Fx_res_filtr_ = Fx_ - Fx_filt_;
        Fx_filt_ = Fx_filt_ + alpha_Fx_ * Fx_res_filtr_;
        Fx_p_filt_ = Fx_p_filt_ + (beta_Fx_ * Fx_res_filtr_) / Ts_;

        Tz_filt_ = Tz_filt_ + (Tz_p_filt_ * Ts_);
        Tz_res_filtr_ = Tz_ - Tz_filt_;
        Tz_filt_ = Tz_filt_ + alpha_Tz_ * Tz_res_filtr_;
        Tz_p_filt_ = Tz_p_filt_ + (beta_Tz_ * Tz_res_filtr_) / Ts_;

        // 3. Cálculo de aceleraciones derivadas de la admitancia
        uk_dot_ = Fx_filt_ / mu - du * u_ / mu;
        wk_dot_ = Tz_filt_ / mw - dw * w_ / mw;

        // 4. Errores de seguimiento cinemático
        double u_tilde = uk_ - u_;
        double w_tilde = wk_ - w_;

        Eigen::Vector2d v_tilde(u_tilde, w_tilde);

        // 5. Superficies de deslizamiento y Matriz G
        double sigma_1 = uk_dot_ + Ku1 * std::tanh(Ku2 * u_tilde);
        double sigma_2 = wk_dot_ + Kw1 * std::tanh(Kw2 * w_tilde);

        Eigen::Matrix<double, 2, 6> G;
        G << sigma_1, 0, -pow(w_, 2), u_, 0, 0,
             0, sigma_2, 0, 0, u_ * w_, w_;

        double epsilon_u = 0.012;//0.03; 
        double epsilon_w = 0.0294;//0.0735; 

        // 6. Ley de Actualización 
        // Si el robot está inactivo O la adaptación está apagada, forzamos los parámetros nominales.
        if (is_inactive || select_adaptation_ == 0) {
            theta_hat_ = theta_;
        } else {
            // Solo si está activo intentamos adaptar
            if (std::abs(u_tilde) > epsilon_u || std::abs(w_tilde) > epsilon_w) {
                
                // Ecuación 17
                Eigen::Vector<double, 6> theta_hat_dot = 
                    gamma_inv_ * (G.transpose() * v_tilde) - 
                    gamma_inv_ * (Gamma_ * theta_hat_); 
                
                // Integración
                theta_hat_ = theta_hat_ + theta_hat_dot * Ts_;
            }
        }

        // 7. Publicar resultados
        auto msg_out = std_msgs::msg::Float64MultiArray();
        for (int i = 0; i < 6; ++i) {
            msg_out.data.push_back(theta_hat_(i));
        }
        pub_params_->publish(msg_out);
    }

    // Tasa de muestreo
    const double Ts_ = 0.01; 
    int8_t select_adaptation_ = 0;

    // Variables de estado
    double u_ = 0.0, w_ = 0.0;
    double du = 10.0, dw = 10.0, mu = 1.0, mw = 1.0; 
    double Fx_ = 0.0, Tz_ = 0.0;

    // Variables para el filtro Alfa-Beta
    double Fx_filt_ = 0.0, Fx_p_filt_ = 0.0, Fx_res_filtr_ = 0.0;
    double Tz_filt_ = 0.0, Tz_p_filt_ = 0.0, Tz_res_filtr_ = 0.0;
    double alpha_Fx_, beta_Fx_, alpha_Tz_, beta_Tz_;

    // Variables para la fuerza de soporte y lógica de reinicio
    double Fz_ = 0.0;
    bool was_reset_logged_ = false;
    
    // Umbrales ajustables empíricamente basados en el ruido de los sensores reales
    const double threshold_u_ = 0.02; // [m/s] Tolerancia a ruido de velocidad lineal
    const double threshold_w_ = 0.02; // [rad/s] Tolerancia a ruido de velocidad angular
    const double threshold_Fz_ = 5.0; // [N] Tolerancia a fuerzas fantasma en vacío

    // Variables de referencia cinemática (Nomenclatura corregida)
    double uk_ = 0.0, wk_ = 0.0;
    double uk_dot_ = 0.0, wk_dot_ = 0.0;

    // Ganancias y Parámetros
    Eigen::Vector<double, 6> theta_;
    Eigen::Vector<double, 6> theta_hat_;
    Eigen::DiagonalMatrix<double, 6> gamma_inv_;
    Eigen::DiagonalMatrix<double, 6> Gamma_;

    double Ku1, Ku2, Kw1, Kw2;
    double lambda_u, lambda_w;

    // ROS 2 Interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_force_; 
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_force_raw_; 
    rclcpp::Subscription<jt_admittance_msgs::msg::AdmittanceParams>::SharedPtr sub_admittance_params_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_adaptation_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_params_;
    builtin_interfaces::msg::Time latest_odom_stamp_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_kin_ref_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParameterEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}