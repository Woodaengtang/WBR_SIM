#ifndef POW_CORE_H
#define POW_CORE_H

#include <iostream>
#include <vector>
#include <chrono>
#include <eigen3/Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "wbr_pow/msg/WbrControl.hpp"
#include "wbr_pow/msg/WbrComCal.hpp"

#define CR2WR 2
#define CL2WL 3

class PowCore : public rclcpp::Node {
    private:
        // Subscribing message from joystick node in gazebo (v_d, yaw_rate_d, h_d, phi_d)
        rclcpp::Subscription<std_msgs::msg::WbrWheel>::SharedPtr sub_right_input_;
        rclcpp::Subscription<std_msgs::msg::WbrWheel>::SharedPtr sub_left_input_;
        rclcpp::Subscription<sensor_msgs/msg/Imu>::SharedPtr sub_imu_;
        rclcpp::Subscription<sensor_msgs/msg/JointState>::SharedPtr sub_encoder_;
        // Publishing desired theta message to HR controller
        // Publishing desired h_d to LQR VYP controller

        void core_init();
        void imu_callback();
        void encoder_callback();
        void right_input_callback();
        void left_input_callback();
        void joystick_callback();

        bool flag_joystick;
        bool flag_imu;
        bool flag_encoder_r;
        bool flag_encoder_l;
        bool flag_input;

        // Calculate Helper
        float cos_theta, sin_theta, cos_theta_2, sin_theta_2, sin_cos_theta, h_2;
        float theta_dot_2, psi_dot_2;

        Eigen::Matrix3f I_B_LW;  // Left Wheel Inertia Tensor (constant)
        Eigen::Matrix3f I_B_RW;  // Right Wheel Inertia Tensor (constant)

        Eigen::Matrix3f I_B_B;   // Body Inertia Tensor (need to be calculated)
        Eigen::Vector3f p_bcom;  // CoM offset (need to be calculated)

        // Kinematic properties
        float a, b, l1, l2, l3, l4, l5, sqr_fwd;  // for kinematics
        Eigen::Vector2f theta_hips;
        float angle_EDF, AB;

        // For CoM calculator
        Eigen::Vector2f theta_As;
        Eigen::Vector2f theta_Bs;
        Eigen::Vector2f theta_ks;
        Eigen::Matrix<float, 3, 7> p_vecs, c_vecs;
        Eigen::Matrix<float, 7, 1> m_vecs;
        std::vector<Eigen::Matrix3f> R_matrices;
        std::vector<Eigen::Matrix3f> IG_matrices;
        float theta_eq;

        // Dynamic properties and Gradient
        Eigen::Matrix3f M;
        Eigen::Matrix3f M_inv;
        Eigen::Matrix3f dM_dtheta;
        Eigen::Matrix3f dnle_dqdot;
        Eigen::Vector3f nle;
        Eigen::Vector3f dnle_dtheta;
        Eigen::Matrix<float, 3, 2> B;  // input matrix
        Eigen::Matrix<float, 4, 4> fx;
        Eigen::Matrix<float, 4, 2> fu;

        Eigen::Matrix<float, 2, 1> u;        // input vector
        float theta, theta_dot, v, psi_dot;  // state variables
        Eigen::Matrix<float, 4, 1> f;
        float yaw_rate_d, theta_dot_d, h_d, phi_d;  // h (m), phi(degree)
        const float g = 9.80665f;
        float m_B, m_LW, m_RW, L, R;

        Eigen::Matrix<float, 4, 4> P;       // 단위 행렬
        Eigen::Matrix<float, 4, 4> P_pred;  // 단위 행렬
        const Eigen::DiagonalMatrix<float, 8> R_cov;
        const Eigen::DiagonalMatrix<float, 4> Q_cov;
        Eigen::Matrix<float, 4, 4> F_mat;               // Jacobian of Dynamic model
        Eigen::Matrix<float, 8, 4> H;                   // Jacobian of Observation model
        Eigen::Matrix<float, 4, 8> K_mat;               // Kalman Gain

        Eigen::Matrix<float, 4, 1> x, x_pred;  // state
        Eigen::Matrix<float, 8, 1> z, h_obs;   // measurement

        void get_theta_eq();
        Eigen::Vector2f get_theta_hips()
        bool prepare_state_prediction();
        void prepare_calculate();
        void calculate_M();
        bool calculate_Minv();

        void predict_state();
        void calculate_com_and_inertia();
        void calculate_nle();
        void calculate_dM_dtheta();
        void calculate_dnle_dtheta();
        void calculate_dnle_dqdot();
        void calculate_fx();
        void calculate_fu();
        inline Eigen::Matrix<float, 3, 3> rotation_matrix_y(float theta);
        void solve_forward_kinematics();
        void solve_inverse_kinematics();

        void predict_measurement();
        void update();
        bool predict();
        bool estimate_state(Eigen::Matrix<float, 4, 1> &x_, const Eigen::Matrix<float, 8, 1> &z_);

    public:
        PowCore();
};


#endif