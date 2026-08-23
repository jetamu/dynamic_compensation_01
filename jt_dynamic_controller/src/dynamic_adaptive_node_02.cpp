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

        // Definir ganancias del usuario
        gamma_inv_.diagonal() << 0.1, 2.0, 2.0, 1.0, 2.0, 2.0;            
        Gamma_.diagonal() << 0.001, 0.0001, 0.0001, 0.0001, 0.001, 0.0001;          
        k_u_ = 0.0;
        k_w_ = 0.0;

        Ku1 = 0.6;
        Ku2 = 0.5/Ku1;
        Kw1 = 1.47;
        Kw2 = 2.0/Kw1; 

        // Suscriptores
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                u_ = msg->twist.twist.linear.x;
                w_ = msg->twist.twist.angular.z;
                latest_odom_stamp_ = msg->header.stamp;
            });

        sub_wrench_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "force/decomposed", 10, [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
                Fx_ = msg->wrench.force.x * 9.8;
                Tz_ = -msg->wrench.torque.z * 9.8;
            });

        sub_admittance_params_ = this->create_subscription<jt_admittance_msgs::msg::AdmittanceParams>(
            "admittance_modulator/admittance_params", 10, 
            [this](const jt_admittance_msgs::msg::AdmittanceParams::SharedPtr msg) { 
                mu_ = msg->m_u; 
                du_ = msg->d_u; 
                mw_ = msg->m_w; 
                dw_ = msg->d_w; 
            });

        sub_adaptation_ = this->create_subscription<std_msgs::msg::Int8>(
            "/hmi_controller/dynamic_adaptation", 10,
            [this](const std_msgs::msg::Int8::SharedPtr msg) {
                // Detectar flanco de subida (de 0 a 1)
                if (this->select_adaptation_ == 0 && msg->data == 1) {
                    this->theta_hat_ = this->theta_; // REINICIO A NOMINAL
                    RCLCPP_INFO(this->get_logger(), "Adaptación ACTIVADA. Parámetros reiniciados a valores nominales.");
                } 
                else if (this->select_adaptation_ == 1 && msg->data == 0) {
                    this->theta_hat_ = this->theta_; // FORZAR NOMINAL AL APAGAR
                    RCLCPP_INFO(this->get_logger(), "Adaptación DESACTIVADA.");
                }
                this->select_adaptation_ = msg->data;
            });

        sub_kin_ref_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "admittance_control/cmd_vel", 10,
            [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
                this->u_ref_ = msg->twist.linear.x;
                this->w_ref_ = msg->twist.angular.z;
            });
        
        // Publicador de los nuevos parámetros
        pub_params_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("adaptive_params", 10);

        // Bucle de control a 100Hz (Ts = 0.01)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ParameterEstimatorNode::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Dynamic adaptative node started.");
    }

private:
    void control_loop() {
        // Prevención de división por cero
        if (mu_ <= 0.001 || mw_ <= 0.001) return;

        // --- NUEVO 1: Calcular aceleraciones derivadas de la referencia ---
        double u_ref_dot = (u_ref_ - u_ref_prev_) / Ts_;
        double w_ref_dot = (w_ref_ - w_ref_prev_) / Ts_;

        // --- NUEVO 2: Filtro Pasa-Bajas (Usando los valores de MATLAB) ---
        double alpha_u = Ts_ / (tau_u_ + Ts_);
        double alpha_w = Ts_ / (tau_w_ + Ts_);
        
        double u_ref_filtered = u_ref_filtered_prev_ + alpha_u * (u_ref_ - u_ref_filtered_prev_);
        double w_ref_filtered = w_ref_filtered_prev_ + alpha_w * (w_ref_ - w_ref_filtered_prev_);

        // 3. Errores (Usando las referencias filtradas)
        double u_tilde = u_ref_filtered - u_;
        double w_tilde = w_ref_filtered - w_;

        Eigen::Vector2d v_tilde(u_tilde, w_tilde);

        // 4. Superficies de deslizamiento (Sigma)
        double sigma_1 = u_ref_dot + Ku1 * std::tanh(Ku2 * u_tilde);
        double sigma_2 = w_ref_dot + Kw1 * std::tanh(Kw2 * w_tilde);

        // 5. Matriz G (Regresor)
        Eigen::Matrix<double, 2, 6> G;
        G << sigma_1, 0, -pow(w_, 2), u_, 0, 0,
             0, sigma_2, 0, 0, u_ * w_, w_;

        // --- NUEVO 3: Reemplaza estos valores con la Regla 3-Sigma calculada en MATLAB ---
        double epsilon_u = 0.041; // REEMPLAZAR con el valor exacto de epsilon_u_dyn_3s
        double epsilon_w = 0.065; // REEMPLAZAR con el valor exacto de epsilon_w_dyn_3s

        // 6. Ley de Actualización con Modulación y Zona Muerta
        if (select_adaptation_ == 1) {
            
            // --- NUEVO 4: Atenuación de la matriz Gamma según la aceleración ---
            double factor_atenuacion_u = std::exp(-beta_ * std::abs(u_ref_dot));
            double factor_atenuacion_w = std::exp(-beta_ * std::abs(w_ref_dot));
            
            Eigen::DiagonalMatrix<double, 6> Gamma_inv_modulada = gamma_inv_;
            
            // ¡IMPORTANTE! Asocia la atenuación a los índices correctos de theta.
            // Aquí asumo que theta 0, 1, 2 son dinámicas lineales y 3, 4, 5 son angulares.
            // Si el orden en tu modelo matemático es diferente, ajusta estos índices.
            Gamma_inv_modulada.diagonal()[0] *= factor_atenuacion_u;
            Gamma_inv_modulada.diagonal()[1] *= factor_atenuacion_u;
            Gamma_inv_modulada.diagonal()[2] *= factor_atenuacion_u;
            
            Gamma_inv_modulada.diagonal()[3] *= factor_atenuacion_w;
            Gamma_inv_modulada.diagonal()[4] *= factor_atenuacion_w;
            Gamma_inv_modulada.diagonal()[5] *= factor_atenuacion_w;

            // --- NUEVO 5: Zona Muerta + Modificación Sigma estricta ---
            if (std::abs(u_tilde) > epsilon_u || std::abs(w_tilde) > epsilon_w) {
                
                // Ley adaptativa: (Ganancia Modulada * Regresor) - (Fuga hacia el parámetro Nominal)
                Eigen::Vector<double, 6> theta_hat_dot = 
                    Gamma_inv_modulada * (G.transpose() * v_tilde) - 
                    Gamma_inv_modulada * (Gamma_ * (theta_hat_ - theta_)); 
                
                // Integración de Euler
                theta_hat_ = theta_hat_ + theta_hat_dot * Ts_;
            }
        } else {
            // Si está desactivado, nos aseguramos de que se mantenga en los valores base
            theta_hat_ = theta_;
        }

        // 7. Publicar resultados
        auto msg_out = std_msgs::msg::Float64MultiArray();
        for (int i = 0; i < 6; ++i) {
            msg_out.data.push_back(theta_hat_(i));
        }
        pub_params_->publish(msg_out);

        // Actualizar variables de estado (memoria)
        u_ref_prev_ = u_ref_;
        w_ref_prev_ = w_ref_;
        u_ref_filtered_prev_ = u_ref_filtered;
        w_ref_filtered_prev_ = w_ref_filtered;
    }

    // Tasa de muestreo
    const double Ts_ = 0.01; 
    int8_t select_adaptation_ = 0;

    // --- NUEVO: Constantes de tiempo y estado del filtro ---
    const double tau_u_ = 0.250; // Calculado vía Mínimos Cuadrados (Línea recta)
    const double tau_w_ = 0.265; // Calculado vía Mínimos Cuadrados (Giros)
    double u_ref_filtered_prev_ = 0.0;
    double w_ref_filtered_prev_ = 0.0;
    
    // --- NUEVO: Factor beta para sensibilidad de atenuación ---
    const double beta_ = 2.0; 

    // Variables de estado
    double u_ = 0.0, w_ = 0.0;
    double Fx_ = 0.0, Tz_ = 0.0;
    double du_ = 10.0, dw_ = 10.0, mu_ = 1.0, mw_ = 1.0; 
    double u_ref_prev_ = 0.0, w_ref_prev_ = 0.0;

    // Variables para almacenar las velocidades de referencia leídas
    double u_ref_ = 0.0;
    double w_ref_ = 0.0;

    // Ganancias y Parámetros
    double k_u_, k_w_;
    Eigen::Vector<double, 6> theta_;
    Eigen::Vector<double, 6> theta_hat_;
    Eigen::DiagonalMatrix<double, 6> gamma_inv_;
    Eigen::DiagonalMatrix<double, 6> Gamma_;

    double Ku1, Ku2, Kw1, Kw2;

    // ROS 2 Interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench_;
    rclcpp::Subscription<jt_admittance_msgs::msg::AdmittanceParams>::SharedPtr sub_admittance_params_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_adaptation_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_params_;
    builtin_interfaces::msg::Time latest_odom_stamp_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_kin_ref_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParameterEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}