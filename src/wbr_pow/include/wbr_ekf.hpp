#ifndef EKF_HPP
#define EKF_HPP

#include <Eigen/Dense>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "wbr_controller/msg/WbrState.hpp"
#include "wbr_controller/msg/WbrComCal.hpp"
#include "wbr_controller/msg/WbrControl.hpp"

#define CR2WR 2
#define CL2WL 3
#define G 9.80665
#define R 0.072
#define L 0.123

class WbrEKF : public rclcpp::Node {
    private:
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;            // Subscribing imu sensor from gazebo wbr model
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;   // Subscribing joint states from gazebo wbr model
        rclcpp::Subscription<std_msgs::msg::WbrComCal>::SharedPtr sub_com_cal_;     // Subscribing com and inertia tensor from com_calculator
        rclcpp::Subscription<std_msgs::msg::WbrControl>::SharedPtr sub_h_d_;
        rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr sub_wheel_right_;   // Subscribing controlled input
        rclcpp::Subscription<geometry_msgs::msg::Wrench>::SharedPtr sub_wheel_left_;   // Subscribing controlled input
        rclcpp::Publisher<std_msgs::msg::WbrState>::SharedPtr pub_state_;           // Publishing wbr states from extended Kalman filter

        Eigen::Matrix<float, 4, 4> P;                       // Covariance matrix
        Eigen::Matrix<float, 4, 4> P_pred;                  // Predicted covariance
        static const Eigen::DiagonalMatrix<float, 8> R_cov; // Measurement covariance
        static const Eigen::DiagonalMatrix<float, 4> Q_cov; // Prediction covariance
        Eigen::Matrix<float, 4, 4> F_mat;                   // Jacobian of dynamic model
        Eigen::Matrix<float, 8, 4> H;                       // Jacobian of Observation model
        Eigen::Matrix<float, 4, 8> K_mat;                   // Kalman Gain
        Eigen::Matrix<float, 4, 1> x;                       // estimated state
        Eigen::Matrix<float, 4, 1> x_pred;                  // predicted state
        Eigen::Matrix<float, 8, 1> z;                       // measurement
        Eigen::Matrix<float, 8, 1> h_obs;                   // measurement
        Eigen::Matrix<float, 6, 1> imu_raw;

        float h = 0.2;                                      // temporal desired height

        float wheel_right_encoder;
        float wheel_left_encoder;

        float wheel_right_torque;
        float wheel_left_torque;

        void initialize_ekf();
        void imu_callback();
        void joint_callback();
        void com_callback();
        void control_callback();
        void wheel_right_callback();
        void wheel_left_callback();
        void output_callback();
        
        void priori_state_estimation();             //
        void priori_covariance_computation();       //
        void jacobian_computation();                //
        void kalman_gain();                         //
        bool posterior_state_estimation();          //
        void posterior_covariance_computation();    //

    public:
        WbrEKF();                   // Initialization of covariance and states
};

#endif // EKF_HPP