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

        // Definir ganancias del usuario (¡Ajustar estos valores empíricamente!)
        gamma_inv_.diagonal() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;            // 10
        Gamma_.diagonal() << 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001;          // 0.001
        k_u_ = 0.0;
        k_w_ = 0.0;

        Ku1 = 0.6;
        Ku2 = 3.0/Ku1;
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
        //pub_kin_ref_vel_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("kinematic_reference/cmd_vel", 10);

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

        // 1. Modelo de Admitancia
        //double u_ref_dot = (Fx_ - du_ * u_ref_prev_) / mu_;
        //double w_ref_dot = (Tz_ - dw_ * w_ref_prev_) / mw_;

        //double u_ref_dot = (Fx_ > 0.1) ? (Fx_ - du_ * u_) / mu_ : 0.0;
        //double w_ref_dot = (Tz_ > 0.5) ? (Tz_ - dw_ * w_) / mw_ : 0.0;

        double u_ref_dot = (Fx_ - du_ * u_) / mu_;
        double w_ref_dot = (Tz_ - dw_ * w_) / mw_;

        /*double u_ref = u_ref_prev_ + u_ref_dot * Ts_;
        double w_ref = w_ref_prev_ + w_ref_dot * Ts_;*/

        /*double u_ref = (Fx_ + (mu_ * u_ref_prev_/Ts_)) / ((mu_/Ts_)+du_);
        double w_ref = (Tz_ + (mw_ * w_ref_prev_/Ts_)) / ((mw_/Ts_)+dw_);*/

        // 2. Errores
        double u_tilde = u_ref_ - u_;
        double w_tilde = w_ref_ - w_;

        Eigen::Vector2d v_tilde(u_tilde, w_tilde);

        // 3. Sigma 1 y Sigma 2
        double sigma_1 = u_ref_dot + Ku1 * std::tanh(Ku2*u_tilde);
        double sigma_2 = w_ref_dot + Kw1 * std::tanh(Kw2*w_tilde);

        // 4. Matriz G
        Eigen::Matrix<double, 2, 6> G;
        G << sigma_1, 0, -pow(w_, 2), u_, 0, 0,
             0, sigma_2, 0, 0, u_ * w_, w_;

        // 5. Ley de actualización (Retorno a los parámetros nominales identificados)
        //Eigen::Vector<double, 6> theta_tilde = theta_hat_ - theta_;

        // 5. Ley de actualización con ZONA MUERTA (Deadzone) SEPARADA
        //Eigen::Vector<double, 6> theta_hat_dot = Eigen::Vector<double, 6>::Zero();
        
        // Umbrales independientes (Ajustar según el ruido en reposo de la odometría)
        // Como sugerencia inicial: ~2% a 5% de sus velocidades máximas respectivas.
        //double epsilon_u = 0.015; // Umbral para error lineal (max 0.6 m/s)
        //double epsilon_w = 0.035; // Umbral para error angular (max 1.47 rad/s)

        // Se activa la adaptación si CUALQUIERA de los errores supera su umbral
        /*if (std::abs(u_tilde) > epsilon_u || std::abs(w_tilde) > epsilon_w) {
            // Ecuación 17: Actualización adaptativa con modificación sigma
            theta_hat_dot = gamma_inv_ * (G.transpose() * v_tilde) - gamma_inv_ * (Gamma_ * theta_hat_);
        }*/

        /*Eigen::Vector<double, 6> theta_hat_dot = 
            gamma_inv_ * (G.transpose() * v_tilde); */
            //- gamma_inv_ * (Gamma_ * theta_hat_);

        // Umbrales independientes
        double epsilon_u = 0.005; // Ajustar según ruido de odometría lineal
        double epsilon_w = 0.010; // Ajustar según ruido de odometría angular

        // --- NUEVA LÓGICA DE INTEGRACIÓN ---
        if (select_adaptation_ == 1) {
            if (std::abs(u_tilde) > epsilon_u || std::abs(w_tilde) > epsilon_w) {
                // 5. Ley de actualización
                Eigen::Vector<double, 6> theta_hat_dot = gamma_inv_ * (G.transpose() * v_tilde) - gamma_inv_ * (Gamma_ * theta_hat_); 
                // 6. Integración de Euler
                theta_hat_ = theta_hat_ + theta_hat_dot * Ts_;
            }
        } else {
            // Si está desactivado, nos aseguramos de que se mantenga en los valores fijos
            theta_hat_ = theta_;
        }
        

        // 6. Integración de Euler para obtener los nuevos parámetros
        //theta_hat_ = theta_hat_ + theta_hat_dot * Ts_;

        // 7. Publicar resultados
        auto msg_out = std_msgs::msg::Float64MultiArray();
        for (int i = 0; i < 6; ++i) {
            msg_out.data.push_back(theta_hat_(i));
        }
        pub_params_->publish(msg_out);

        // Actualizar variables de estado
        u_ref_prev_ = u_ref_;
        w_ref_prev_ = w_ref_;

        // --- NUEVO: Publicar velocidades cinemáticas de referencia en tipo TwistStamped ---
        /*auto msg_kin_ref = geometry_msgs::msg::TwistStamped();
        msg_kin_ref.header.stamp = latest_odom_stamp_;
        msg_kin_ref.header.frame_id = "base_link";
        msg_kin_ref.twist.linear.x = u_ref;
        msg_kin_ref.twist.angular.z = w_ref;
        pub_kin_ref_vel_->publish(msg_kin_ref);*/

    }

    // Tasa de muestreo
    const double Ts_ = 0.01; 
    int8_t select_adaptation_ = 0;

    // Variables de estado
    double u_ = 0.0, w_ = 0.0;
    double Fx_ = 0.0, Tz_ = 0.0;
    double du_ = 10.0, dw_ = 10.0, mu_ = 1.0, mw_ = 1.0; // Valores default seguros
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
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_adaptation_; // Suscriptor bandera
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_params_;
    //rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_kin_ref_vel_;
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