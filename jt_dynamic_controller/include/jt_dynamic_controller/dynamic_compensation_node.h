#ifndef DYNAMIC_COMPENSATION_NODE_H_
#define DYNAMIC_COMPENSATION_NODE_H_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp> 
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <Eigen/Dense>
#include <cmath>
#include <std_msgs/msg/float64_multi_array.hpp>
#include "std_msgs/msg/int8.hpp"
#include "jt_admittance_msgs/msg/admittance_params.hpp"


class dynamic_compensation_class : public rclcpp::Node
{
public:
    dynamic_compensation_class();
    ~dynamic_compensation_class();

    void stop_robot();

private:
    void callback(const nav_msgs::msg::Odometry::ConstSharedPtr& msg1, 
                  const geometry_msgs::msg::WrenchStamped::ConstSharedPtr& msg2);

    double ts_; 
    double u_, w_;
    double Fx_, Tz_;
    double mu, mw, du, dw;
    
    Eigen::Matrix<double, 6, 1> chi_;
    Eigen::Matrix2d M_;

    double Fx_filt_, Fx_p_filt_, Fx_res_filtr_;
    double Tz_filt_, Tz_p_filt_, Tz_res_filtr_;
    double alpha_Fx_, beta_Fx_;
    double alpha_Tz_, beta_Tz_;

    double Ku1, Ku2, Kw1, Kw2;
    double lambda_u, lambda_w;
    double uk_, wk_, uk_dot_, wk_dot_, uk_ant_, wk_ant_;

    rclcpp::Subscription<jt_admittance_msgs::msg::AdmittanceParams>::SharedPtr admittance_params_sub_;
    void admittance_params_Callback(const jt_admittance_msgs::msg::AdmittanceParams::SharedPtr msg);

    // ROS 2 Interfaces
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr dinamic_control_pub_;   
     
    // Suscriptores de message_filters
    message_filters::Subscriber<nav_msgs::msg::Odometry> sub_vel_filtered_;
    message_filters::Subscriber<geometry_msgs::msg::WrenchStamped> sub_force_;

    // Definición de la política de sincronización
    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry, geometry_msgs::msg::WrenchStamped> SyncPolicy;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_sub_;

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_subscription_;
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &parameters);

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr theta_Sub;
    void theta_Callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    /*rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr select_adaptation_sub_;
    void select_adaptation_Callback(const std_msgs::msg::Int8::SharedPtr msg);*/

    int8_t select_adaptation_ = 0; // 0 por defecto
    Eigen::VectorXd default_chi_;
    Eigen::VectorXd adaptive_chi_;

    // Variable para almacenar el torque virtual en Z
    double virtual_torque_z_ = 0.0;

    // Suscriptor para el Wrench virtual
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr virtual_wrench_sub_;

    // Callback para leer el tópico
    void virtual_wrench_Callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

    // Suscriptor para las velocidades cinemáticas de referencia
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr kin_ref_sub_;
    
    // Callback para actualizar uk_ y wk_
    void kin_ref_Callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    // Añadir en la sección privada/pública de tu .h
    float user_weight;
    float min_force_z;
    double force_left_z = 0.0;
    double force_right_z = 0.0;
    bool is_active_ = false;

    // Suscriptor para leer las fuerzas verticales
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_raw_sub_;
    void force_raw_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
};

#endif // DYNAMIC_COMPENSATION_NODE_H_