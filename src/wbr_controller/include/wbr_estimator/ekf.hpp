#ifndef EKF_HPP
#define EKF_HPP

#include <Eigen/Dense>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "wbr_controller/msg/WbrState.hpp"

class EKF : public rclcpp::Node {
    private:
        Eigen::Matrix<float, 4, 4> P;                 // Covariance matrix
        Eigen::Matrix<float, 4, 4> P_pred;            // Predicted covariance
        const Eigen::DiagonalMatrix<float, 8> R_cov;  // Measurement covariance
        const Eigen::DiagonalMatrix<float, 4> Q_cov;  // Prediction covariance
        Eigen::Matrix<float, 4, 4> F_mat;             // Jacobian of dynamic model
        Eigen::Matrix<float, 8, 4> H;                 // Jacobian of Observation model
        Eigen::Matrix<float, 4, 8> K_mat;             // Kalman Gain
        Eigen::Matrix<float, 4, 1> x;                 // estimated state
        Eigen::Matrix<float, 4, 1> x_pred;            // predicted state
        Eigen::Matrix<float, 8, 1> z;                 // measurement
        Eigen::Matrix<float, 8, 1> h_obs;             // measurement

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;            // Imu subscribed from gazebo wbr model
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_;   // Joint state subscribed from gazebo wbr model
        rclcpp::Publisher<std_msgs::msg::WbrState>::SharedPtr pub_state_;           // Estimated state from extended Kalman filter published

        void priori_state_estimation();
        void calculate_priori_covariance();
        void calculate_jacobian();
        void calculate_kalman_gain();
        bool posterior_state_estimation();
        void calculate_posterior_covariance();

    public:
        EKF();                      // Initialization of covariance and states
        void print_all_ekf_info();  // Print all extended Kalman filter's information
        void reset_ekf();           // Reset ekf to initialized values
};

#endif // EKF_HPP
