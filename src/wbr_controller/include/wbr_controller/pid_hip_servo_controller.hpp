#ifndef PD_HIP_SERVO_CONTROLLER_HPP
#define PD_HIP_SERVO_CONTROLLER_HPP

#include <iostream>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "wbr_controller/msg/WbrState.hpp"

class PidHipServoController : public rclcpp::Node {
    private:
        rclcpp::Subscription<std_msgs::msg::WbrState>::SharedPtr sub_state_;        // State subscription from efk
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_thigh_left_;   // Publish left hip motor's torque
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_thigh_right_;  // Publish right hip motor's torque
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_main_body_;    // Publish antitorque to main body link

        void calculate_antitorque_to_main_body();   //
        void hip_servo_constraints();               //

        Eigen::Vector2f theta_hips;                 //
        static const Eigen::Vector3f pid_gain;
        
        static const float left_hip_servo_min;      //
        static const float left_hip_servo_max;      //
        static const float left_hip_servo_mid;      //
        static const float right_hip_servo_min;     //
        static const float right_hip_servo_max;     //
        static const float right_hip_servo_mid;     //

    public:
        PidHipServoController();
};

#endif