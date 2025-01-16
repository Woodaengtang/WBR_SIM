#include "wbr_ekf.hpp"

WbrEKF::WbrEKF() : Node("wbr_ekf") {
    initialize_ekf();
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_plugin/out", 10, std::bind(&WbrEKF::imu_callback, this, std::placeholders::_1));
    sub_joint_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&WbrEKF::joint_callback, this, std::placeholders::_1));
    // sub_com_cal_ = this->create_subscription<std_msgs::msg::WbrComCal>(
    //     "topic_name_later", 10, std::bind(&WbrEKF::com_callback, this, std::placeholders::_1));
    sub_h_d_ = this->create_subscription<std_msgs::msg::WbrControl>(
        "topic name later", 10, std::bind(&WbrEKF::control_callback, this, std::placeholders::_1));
    sub_wheel_right_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "/wheel_right_force/gazebo_ros_force", 10, std::bind(&WbrEKF::wheel_right_callback, this, std::placeholders::_1));
    sub_wheel_left_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "/wheel_left_force/gazebo_ros_force ", 10, std::bind(&WbrEKF::wheel_left_callback, this, std::placeholders::_1));
    
    pub_state_ = this->create_publisher<std_msgs::msg::WbrState>("set name later", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(8), std::bind(&WbrEKF::output_callback , this));
}

void WbrEKF::initialize_ekf(){
    P = Eigen::Matrix<float, 4, 4>::Identity() * 1.f;
    P_pred = Eigen::Matrix<float, 4, 4>::Identity() * 1.f;
    R_cov((Eigen::Matrix<float, 8, 1>() << 4e-1f, 4.f, 4e-1f, 1e-2f, 1e-4f, 1e-2f, 0, 0).finished().asDiagonal()),  // sensor noise covariance
    Q_cov((Eigen::Matrix<float, 4, 1>() <<  0, 1.f, 1.f, 1.f).finished().asDiagonal())                              // process noise covariance
    F_mat.setZero();
    H.setZero();
    K_mat.setZero();
    x.setZero();
    x_pred.setZero();
    z.setZero();
    h_obs.setZero();
}

void WbrEKF::imu_callback(const sensor_msgs::msg::Imu msg){
    imu_raw << msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z,
            msg->angular_velocity.x,
            msg->angular_velocity.y,
            msg->angular_velocity.z;
}

void WbrEKF::joint_callback(const sensor_msgs::msg::JointState msg){
    wheel_right_encoder = msg->velocity[CR2WR];
    wheel_left_encoder = msg->velocity[CL2WL];
}

void WbrEKF::com_callback(const std_msgs::msg::WbrComCal msg){
    // msg->ibb;
    // msg->com;
    // msg->theta_eq;
}

void WbrEKF::control_callback(const std_msgs::msg::WbrControl msg){
    // h = msg->h_d;
}

void WbrEKF::wheel_right_callback(const geometry_msgs::msg::Wrench msg){
    wheel_right_torque = msg->torque.y;
}

void WbrEKF::wheel_left_callback(const geometry_msgs::msg::Wrench msg){
    wheel_left_torque = msg->torque.y;
}

void WbrEKF::output_callback(){
    auto output_msg_ = std_msgs::msg::WbrState();
    priori_state_estimation();
    priori_covariance_computation();
    jacobian_computation();
    kalman_gain();
    posterior_state_estimation();
    posterior_covariance_computation();
    pub_state_->publish(output_msg_);
}

void WbrEKF::priori_state_estimation() {
    // float theta_ddot = Pol_ref.f(1);
    // float v_dot = Pol_ref.f(2);
    // float psi_ddot = Pol_ref.f(3);
    float theta_ddot = 0.f;
    float v_dot = 0.f;
    float psi_ddot = 0.f;

    float theta = x_pred(0);
    float theta_dot = x_pred(1);
    float v = x_pred(2);
    float psi_dot = x_pred(3);

    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    float cos_theta_2 = std::pow(cos_theta, 2);
    float sin_theta_2 = std::pow(sin_theta, 2);
    float sin_cos_theta = cos_theta * sin_theta;

    // h_obs calculation(y_hat)
    h_obs(0) = h * theta_ddot + v_dot * cos_theta - G * sin_theta - h * std::pow(psi_dot, 2) * sin_cos_theta;
    h_obs(1) = psi_dot * v + h * psi_ddot * sin_theta + h * psi_dot * theta_dot * cos_theta * 2.0;
    h_obs(2) = -h * std::pow(theta_dot, 2) + G * cos_theta + v_dot * sin_theta - std::pow(psi_dot, 2) * (h - h * cos_theta_2);
    h_obs(3) = -psi_dot * sin_theta;
    h_obs(4) = theta_dot;
    h_obs(5) = psi_dot * cos_theta;
    h_obs(6) = theta_dot - v / R - (L * psi_dot) / R;
    h_obs(7) = -theta_dot + v / R - (L * psi_dot) / R;

    // Jacobian H calculation
    H(0, 0) = std::pow(psi_dot, 2) * (h * sin_theta_2 - h * cos_theta_2) - G * cos_theta - v_dot * sin_theta;
    H(1, 0) = h * psi_ddot * cos_theta - h * psi_dot * theta_dot * sin_theta * 2.0;
    H(2, 0) = v_dot * cos_theta - G * sin_theta - h * std::pow(psi_dot, 2) * sin_cos_theta * 2.0;
    H(3, 0) = -psi_dot * cos_theta;
    // H(4, 0) = 0.0;
    H(5, 0) = -psi_dot * sin_theta;
    // H(6, 0) = 0.0;
    // H(7, 0) = 0.0;

    // H(0, 1) = 0.0;
    H(1, 1) = h * psi_dot * cos_theta * 2.0;
    H(2, 1) = h * theta_dot * -2.0;
    // H(3, 1) = 0.0;
    H(4, 1) = 1.0;
    // H(5, 1) = 0.0;
    H(6, 1) = 1.0;
    H(7, 1) = -1.0;

    // H(0, 2) = 0.0;
    H(1, 2) = psi_dot;
    // H(2, 2) = 0.0;
    // H(3, 2) = 0.0;
    // H(4, 2) = 0.0;
    // H(5, 2) = 0.0;
    H(6, 2) = -1.0 / R;
    H(7, 2) = 1.0 / R;

    H(0, 3) = h * psi_dot * sin_cos_theta * -2.0;
    H(1, 3) = v + h * theta_dot * cos_theta * 2.0;
    H(2, 3) = psi_dot * (h - h * cos_theta_2) * -2.0;
    H(3, 3) = -sin_theta;
    // H(4, 3) = 0.0;
    H(5, 3) = cos_theta;
    H(6, 3) = -L / R;
    H(7, 3) = -L / R;
  }

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WbrEKF>());
    rclcpp::shutdown();
    return 0;
}