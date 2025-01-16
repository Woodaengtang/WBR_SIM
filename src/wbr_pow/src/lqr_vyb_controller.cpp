#include "lqr_vyb_controller.hpp"

LqrVybController::LqrVybController() : Node("lqr_vyb_controller") {
    initialize_gain_set();
    sub_input_ = this->create_subscription<std_msgs::msg::WbrControl>(
        "topic_name_later", 10, std::bind(&LqrVybController::input_callback, this, std::placeholders::_1));
    sub_ekf_ = this->create_subscription<std_msgs::msg::WbrState>(
        "topic_name_later", 10, std::bind(&LqrVybController::ekf_callback, this, std::placeholders::_1));
    sub_com_ = this->create_subscription<std_msgs::msg::WbrComCal>(
        "topic_name_later", 10, std::bind(&LqrVybController::com_callback, this, std::placeholders::_1));
    pub_wheel_left_ = this->create_publisher<geometry_msgs::msg::Wrench>("/wheel_left_force/gazebo_ros_force", 10);
    pub_wheel_right_ = this->create_publisher<geometry_msgs::msg::Wrench>("/wheel_right_force/gazebo_ros_force", 10);
    pub_calf_left_ = this->create_publisher<geometry_msgs::msg::Wrench>("/calf_left_force/gazebo_ros_force", 10);
    pub_calf_right_ = this->create_publisher<geometry_msgs::msg::Wrench>("/calf_right_force/gazebo_ros_force", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(8), std::bind(&LqrVybController::output_callback, this));
}

void LqrVybController::initialize_gain_set(){
    LQR_gain_set = {
        (Eigen::Matrix<float, 2, 4>() << 0.79157211f, 0.08949879f, 0.13324021f, -0.04843267f,
        -0.80162498f, -0.09129509f, -0.13488934f, -0.04909636f).finished(),
        (Eigen::Matrix<float, 2, 4>() << 0.82749528f, 0.09377660f, 0.13284322f, -0.04844744f,
        -0.83769858f, -0.09563814f, -0.13426330f, -0.04915773f).finished(),

        (Eigen::Matrix<float, 2, 4>() << 0.86248546f, 0.09817186f, 0.13272420f, -0.04843592f,
        -0.87284050f, -0.10010141f, -0.13393901f, -0.04919017f).finished(),

        (Eigen::Matrix<float, 2, 4>() << 0.89646446f, 0.10268749f, 0.13287970f, -0.04840456f,
        -0.90697205f, -0.10468726f, -0.13391117f, -0.04920005f).finished(),

        (Eigen::Matrix<float, 2, 4>() << 0.92946708f, 0.10731846f, 0.13327102f, -0.04835495f,
        -0.94013083f, -0.10939071f, -0.13413915f, -0.04918921f).finished(),

        (Eigen::Matrix<float, 2, 4>() << 0.96155309f, 0.11205740f, 0.13385600f, -0.04828783f,
        -0.97237973f, -0.11420460f, -0.13457902f, -0.04915871f).finished(),

        (Eigen::Matrix<float, 2, 4>() << 0.99278639f, 0.11689660f, 0.13459647f, -0.04820401f,
        -1.00378535f, -0.11912147f, -0.13519096f, -0.04910964f).finished(),

        (Eigen::Matrix<float, 2, 4>() << 1.02322897f, 0.12182895f, 0.13545998f, -0.04810470f,
        -1.03441171f, -0.12413443f, -0.13594105f, -0.04904345f).finished(),

        (Eigen::Matrix<float, 2, 4>() << 1.05293953f, 0.12684856f, 0.13642001f, -0.04799157f,
        -1.06431876f, -0.12923772f, -0.13680135f, -0.04896198f).finished(),

        (Eigen::Matrix<float, 2, 4>() << 1.08197396f, 0.13195122f, 0.13745562f, -0.04786668f,
        -1.09356295f, -0.13442718f, -0.13774963f, -0.04886743f).finished(),

        (Eigen::Matrix<float, 2, 4>() << 1.11038720f, 0.13713502f, 0.13855115f, -0.04773247f,
        -1.12219903f, -0.13970087f, -0.13876898f, -0.04876231f).finished(),

        (Eigen::Matrix<float, 2, 4>() << 1.13823694f, 0.14240134f, 0.13969602f, -0.04759179f,
        -1.15028372f, -0.14506002f, -0.13984764f, -0.04864950f).finished(),

        (Eigen::Matrix<float, 2, 4>() << 1.16559157f, 0.14775666f, 0.14088482f, -0.04744880f,
        -1.17788281f, -0.15051066f, -0.14097876f, -0.04853311f).finished(),

        (Eigen::Matrix<float, 2, 4>() << 1.19254970f, 0.15321643f, 0.14211740f, -0.04731278f,
        -1.20508696f, -0.15606684f, -0.14215981f, -0.04842216f).finished()
    };
}


void LqrVybController::input_callback(const std_msgs::msg::WbrControl::SharedPtr msg) {
    vel_d = msg->v_d;
    yaw_rate_d = msg->yaw_rate_d;
    h_d = msg->h_d;
    phi_d = msg->phi_d;
    x_d << vel_d, yaw_rate_d, h_d, phi_d;
}

void LqrVybController::ekf_callback(const std_msgs::msg::WbrState::SharedPtr msg) {
    theta_hat = msg->theta;
    theta_dot_hat = msg->theta_dot;
    vel_hat = msg->vel;
    psi_dot_hat = msg->psi_dot;
    x << theta_hat, theta_dot_hat, vel_hat, psi_dot_hat;
}

void LqrVybController::com_callback(const std_msgs::msg::WbrComCal::SharedPtr msg) {
    theta_eq = msg->theta_eq;
}

void LqrVybController::output_callback(){
    auto wl_msg_ = geometry_msgs::msg::Wrench();
    auto wr_msg_ = geometry_msgs::msg::Wrench();
    auto cl_msg_ = geometry_msgs::msg::Wrench();
    auto cr_msg_ = geometry_msgs::msg::Wrench();

    // set torque input
    gain_computation(h_d);
    input_computation(wr_msg_, wl_msg_);
    antitorque_computation(wr_msg_, wl_msg_, cr_msg_, cl_msg_);

    pub_wheel_left_->publish(wl_msg_);      // torque input to left wheel
    pub_wheel_right_->publish(wr_msg_);     // torque input to right wheel
    pub_calf_left_->publish(cl_msg_);       // antitorque to left calf
    pub_calf_right_->publish(cr_msg_);      // antitorque to right calf
}

void LqrVybController::gain_computation(const float h){
    float temp = (h - HEIGHT_MIN) / INTERPOLATION_STEP;   // devide interpolation section with 10mm length
    int idx = static_cast<int>(temp);       // integer index of the section

    if (idx >= 0 && idx < static_cast<int>(LQR_gain_set.size()) - 1) {
      // interpolation ration computation
      float ratio = temp - idx;
      // interpolating LQR gain
      LQR_gain = LQR_gain_set.at(idx) * (1.0f - ratio) + LQR_gain_set.at(idx + 1) * ratio;
    } else if (idx < 0) {
      LQR_gain = LQR_gain_set.front();
    } else {
      LQR_gain = LQR_gain_set.back();
    }
}

void LqrVybController::input_computation(geometry_msgs::msg::Wrench &wr_msg_, geometry_msgs::msg::Wrench &wl_msg_) {
    wheel_inputs = LQR_gain * (x_d - x);
    // Input saturation
    for (int j = 0; j < 2; j++) {
      if (wheel_inputs(j) > saturation) {
        wheel_inputs(j) = saturation;
      } else if (wheel_inputs(j) < -saturation) {
        wheel_inputs(j) = -saturation;
      }
    }
    wr_msg_.force.x = 0.0;
    wr_msg_.force.y = 0.0;
    wr_msg_.force.z = 0.0;
    wr_msg_.torque.x = 0.0;
    wr_msg_.torque.y = wheel_inputs(0);
    wr_msg_.torque.z = 0.0;

    wl_msg_.force.x = 0.0;
    wl_msg_.force.y = 0.0;
    wl_msg_.force.z = 0.0;
    wl_msg_.torque.x = 0.0;
    wl_msg_.torque.y = wheel_inputs(1);
    wl_msg_.torque.z = 0.0;
}

void LqrVybController::antitorque_computation(const geometry_msgs::msg::Wrench &wr_msg_, 
                                              const geometry_msgs::msg::Wrench &wl_msg_,
                                              geometry_msgs::msg::Wrench &cr_msg_, 
                                              geometry_msgs::msg::Wrench &cl_msg_) {
    cr_msg_.force.x = 0.0;
    cr_msg_.force.y = 0.0;
    cr_msg_.force.z = 0.0;
    cr_msg_.torque.x = 0.0;
    cr_msg_.torque.y = -wl_msg_.torque.y;
    cr_msg_.torque.z = 0.0;

    cl_msg_.force.x = 0.0;
    cl_msg_.force.y = 0.0;
    cl_msg_.force.z = 0.0;
    cl_msg_.torque.x = 0.0;
    cl_msg_.torque.y = -wr_msg_.torque.y;
    cl_msg_.torque.z = 0.0;
}


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LqrVybController>());
    rclcpp::shutdown();
    return 0;
}