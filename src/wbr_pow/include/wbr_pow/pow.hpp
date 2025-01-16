#include <Eigen/Dense>
#include "wbr_parameter/parameters.hpp"

class POW {
    private:
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
        Eigen::Matrix<float, 4, 1> x;        // state
        float theta, theta_dot, v, psi_dot;  // state variables

    public:
        POW();
        void setHR();
        void setState();
        void setInput();
        void get_theta_eq();
        Eigen::Vector2f get_theta_hips();
        bool prepare_state_prediction();
        void predict_state(Eigen::Matrix<float, 4, 1>& x_pred, Eigen::Matrix<float, 4, 4>& F_mat);
        void calculate_com_and_inertia();
        void print_all_variables();
        void solve_inverse_kinematics();

        Eigen::Matrix<float, 4, 1> f;
        float h, phi;  // h (m), phi(degree)
        const float g = 9.80665f;
        float m_B, m_LW, m_RW, L, R;

}