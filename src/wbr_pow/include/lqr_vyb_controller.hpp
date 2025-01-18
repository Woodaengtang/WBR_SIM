#ifndef LQR_WHEEL_SERVO_CONTROLLER_HPP
#define LQR_WHEEL_SERVO_CONTROLLER_HPP

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
// #include <Eigen/Dense>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "wbr_pow/msg/WbrControl.hpp"
#include "wbr_pow/msg/WbrState.hpp"
#include "wbr_pow/msg/WbrComCal.hpp"

#define MAX_TORQUE_COMMAND 100.000  // max torque command
#define HEIGHT_MIN 0.07             // minimum height
#define HEIGHT_MAX 0.2              // maximum height
#define INTERPOLATION_STEP 0.01f    // interpolation step
/**
 * @brief Node for Velocity, Yaw, Balance controller
 * Subscrbing x_d(v_d, yaw_d), theta_eq and x_hat
 * Calculating control input using LQR controller
 * Publishing Wheel Left/Right torque and Calf Left/Right antitorque
 */

class LqrVybController : public rclcpp::Node {
    private:
        rclcpp::Subscription<std_msgs::msg::WbrDesState>::SharedPtr sub_ref_;       // Subscribing desired state
        rclcpp::Subscription<std_msgs::msg::WbrEstState>::SharedPtr sub_ekf_;       // Subscribing estimated states from ekf
        rclcpp::Subscription<std_msgs::msg::WbrPitchEq>::SharedPtr sub_eq_;         // Subscribing equilbrium point from pow_core
        rclcpp::Subscription<std_msgs::msg::WbrHR>::SharedPtr sub_hr_;              // Subscribing hr control input
        
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_wheel_left_;   // Publishing left wheel motor's torque
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_wheel_right_;  // Publishing right wheel motor's torque
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_calf_left_;    // Publishing antitorque to left calf
        rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr pub_calf_right_;   // Publishing antitorque to right calf

        void des_state_callback();
        void ekf_callback();
        void eq_callback();
        void hr_callback();

        void output_callback();

        rclcpp::TimerBase::SharedPtr timer_;
        
        /**
         * @brief wheel_inputs(0) : right wheel motor,
         *        wheel_inputs(1) : left wheel motor
         */
        Eigen::Vector2f wheel_inputs;
        float mainbody_antitorque;

        const float iq_factor = 0.001611328f;                                       // (A/LSB) 3.3 / 2048
        const float torque_constant = 0.7f;                                         // (Nm/A) * reduction ratio
        const float saturation = iq_factor * torque_constant * MAX_TORQUE_COMMAND;  // input saturation

        const float wheel_left_factor = 0.001043224f;
        const float wheel_right_factor = 0.000857902f;

        Eigen::Vector4f x;
        Eigen::Vector4f x_d;

        void initialize_gain_set();
        void gain_computation();
        void input_computation();
        void antitorque_computation();

        //////////////////////////////////////////////////////////
        // LQR gain initialization
        std::vector<Eigen::Matrix<float, 2, 4>> LQR_gain_set;
        Eigen::Matrix<float, 2, 4> LQR_gain;
        Eigen::Matrix<float, 2, 4> mat;
        

    public:
        LqrWheelServoController();
};
#endif