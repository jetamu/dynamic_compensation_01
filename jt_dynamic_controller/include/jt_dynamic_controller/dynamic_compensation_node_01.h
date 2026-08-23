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
    double uk_, wk_, uk_dot_, wk_dot_, uk_ant_, wk_ant_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr du_Sub;
    void du_Callback(const std_msgs::msg::Float32::SharedPtr du_aux);

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr dw_Sub;
    void dw_Callback(const std_msgs::msg::Float32::SharedPtr dw_aux);

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr mu_Sub;
    void mu_Callback(const std_msgs::msg::Float32::SharedPtr mu_aux);

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr mw_Sub;
    void mw_Callback(const std_msgs::msg::Float32::SharedPtr mw_aux);

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

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr select_adaptation_sub_;
    void select_adaptation_Callback(const std_msgs::msg::Int8::SharedPtr msg);

    int8_t select_adaptation_ = 0; // 0 por defecto
    Eigen::VectorXd default_chi_;
    Eigen::VectorXd adaptive_chi_;

    // Variable para almacenar el torque virtual en Z
    double virtual_torque_z_ = 0.0;

    // Suscriptor para el Wrench virtual
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr virtual_wrench_sub_;

    // Callback para leer el tópico
    void virtual_wrench_Callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
};

#endif // DYNAMIC_COMPENSATION_NODE_H_