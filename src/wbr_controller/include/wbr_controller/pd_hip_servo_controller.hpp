#ifndef PD_HIP_SERVO_CONTROLLER_HPP
#define PD_HIP_SERVO_CONTROLLER_HPP

#include <iostream>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "wbr_controller/msg/WbrState.hpp"

class PdHipServoController : public rclcpp::Node {
    private:
        rclcpp::Subscription<std_msgs::msg::WbrState>::SharedPtr sub_state_;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_thigh_left_;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_thigh_right_;
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_main_body_;
        
    public:
        PdHipServoController();
        
};

#endif