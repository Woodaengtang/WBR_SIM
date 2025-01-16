#ifndef LQR_WHEEL_SERVO_CONTROLLER_HPP
#define LQR_WHEEL_SERVO_CONTROLLER_HPP

#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "wbr_controller/msg/WbrState.hpp"

class LqrWheelServoController : public rclcpp::Node {
    private:
        rclcpp::Subscription<std_msgs::msg::WbrState>::SharedPtr sub_state_;        // State subscription from efk
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_wheel_left_;   // Publish left wheel motor's torque
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_wheel_right_;  // Publish right wheel motor's torque
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_calf_left_;    // Publish antitorque to left calf
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_calf_right_;   // Publish antitorque to right calf

        std::vector<Eigen::Matrix<float, 2, 4>> LQR_gain_set;
        Eigen::Matrix<float, 2, 4> LQR_gain;
        /**
         * @brief wheel_inputs(0) : right wheel motor,
         *        wheel_inputs(1) : left wheel motor
         */
        Eigen::Matrix<float, 2, 1> wheel_inputs;
        float mainbody_antitorque;

        static const float iq_factor;
        static const float torque_constant;
        static const float wheel_left_factor;
        static const float wheel_right_factor;
        static const float saturation;
        float theta_d;

        void gain_computation();
        void input_computation();
        void antitorque_computation();

    public:
        LqrWheelServoController();

};
#endif