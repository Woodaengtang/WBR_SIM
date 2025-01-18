#include "pow_core.hpp"
#include "params.hpp"

PowCore::PowCore() : Node("pow_core") {
    core_init();
    // Subscribing message from joystick node in gazebo (v_d, yaw_rate_d, h_d, phi_d)
    sub_input_;    // Subscribing estimated state from ekf
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu_plugin/out", 10, std::bind(&PowCore::imu_callback, this, std::placeholders::_1));
    sub_encoder_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10, std::bind(&PowCore::encoder_callback, this, std::placeholders::_1));
    sub_right_input_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "/wheel_right_force/gazebo_ros_force", 10, std::bind(&PowCore::right_input_callback, this, std::placeholders::_1));
    sub_left_input_ = this->create_subscription<geometry_msgs::msg::Wrench>(
        "/wheel_left_force/gazebo_ros_force", 10, std::bind(&PowCore::left_input_callback, this, std::placeholders::_1));
    
    // Publishing desired theta message to HR controller
    // Publishing desired h_d to LQR VYP controller

    pub_state_ = this->create_publisher<std_msgs::msg::WbrState>("set name later", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(8), std::bind(&WbrEKF::output_callback , this));

}

void PowCore::core_init(){
    const Properties properties_ = createDefaultProperties();

    flag_joystick = false;
    flag_imu = false;
    flag_encoder_r = false;
    flag_encoder_l = false;
    flag_input = false;

    u.setZero();
    x.setZero();
    x_pred.setZero();
    z.setZero();
    theta = 0;
    theta_dot = 0;
    v = 0;
    psi_dot = 0;

    // Scalars
    cos_theta = 0.0f;
    sin_theta = 0.0f;
    cos_theta_2 = 0.0f;
    sin_theta_2 = 0.0f;
    sin_cos_theta = 0.0f;
    h_2 = 0.0f;
    theta_dot_2 = 0.0f;
    psi_dot_2 = 0.0f;

    fx.setZero();
    fx(0, 1) = 1;
    fu.setZero();
    a = properties_.a;
    b = properties_.b;
    l1 = properties_.l1;
    l2 = properties_.l2;
    l3 = properties_.l3;
    l4 = properties_.l4;
    l5 = properties_.l5;
    sqr_fwd = a * a + b * b + l1 * l1 + l2 * l2 - l3 * l3;
    L = properties_.L;
    R = properties_.R;

    angle_EDF = atan(l5 / l4);
    AB = sqrt(a * a + b * b);
    if (R == 0.0f || L == 0.0f) {
        throw std::runtime_error("R or L can't be 0");
    }

    B << 1.0f, -1.0f,
      -1.0f / R, 1.0f / R,
      -L / R, -L / R;

    p_vecs.col(0) << 0.0f, 0.0f, 0.0f;
    p_vecs.col(1) << -0.064951905284f, -0.086f, 0.0375f;
    p_vecs.col(2) << -0.064951905284f, 0.086f, 0.0375f;
    p_vecs.col(3) << 0.0f, -0.081f, 0.0f;
    p_vecs.col(4) << 0.0f, 0.081f, 0.0f;
    R_matrices.resize(7);
    R_matrices[0] = Eigen::Matrix3f::Identity();

    c_vecs.col(0) = properties_.CoM_Body;
    c_vecs.col(1) = properties_.CoM_TAR;
    c_vecs.col(2) = properties_.CoM_TAL;
    c_vecs.col(3) = properties_.CoM_TPR;
    c_vecs.col(4) = properties_.CoM_TPL;
    c_vecs.col(5) = properties_.CoM_CR;
    c_vecs.col(6) = properties_.CoM_CL;

    m_vecs << properties_.m_Body, properties_.m_TAR, properties_.m_TAL, properties_.m_TPR, properties_.m_TPL, properties_.m_CR, properties_.m_CL;
    m_B = m_vecs.sum();
    m_RW = properties_.m_RW;
    m_LW = properties_.m_LW;

    IG_matrices.resize(7);
    IG_matrices[0] = properties_.I_Body;
    IG_matrices[1] = properties_.I_TAR;
    IG_matrices[2] = properties_.I_TAL;
    IG_matrices[3] = properties_.I_TPR;
    IG_matrices[4] = properties_.I_TPL;
    IG_matrices[5] = properties_.I_CR;
    IG_matrices[6] = properties_.I_CL;
    I_B_RW = properties_.I_RW;
    I_B_LW = properties_.I_LW;

    h_obs.setZero();
    F_mat.setZero();
    H.setZero();
    K_mat.setZero();
    P = Eigen::Matrix<float, 4, 4>::Identity() * 1e-1;
    P_pred = Eigen::Matrix<float, 4, 4>::Identity() * 1e-1;
    R_cov = Eigen::DiagonalMatrix<float, 8>((Eigen::Matrix<float, 8, 1>() << 
            1.54239e-3f, 1.93287e-3f, 2.63191e-3f, 3.11351e-6f, 
            4.12642e-6f, 5.37135e-6f, 3.046e-6f, 3.046e-6f).finished());
    Q_cov = Eigen::DiagonalMatrix<float, 4>((Eigen::Matrix<float, 4, 1>() << 
            4e-5f, 1e-4f, 4e-5f, 1e-6f).finished());

}

void PowCore::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
    flag_ekf = true;
    z.block<6, 1>(0, 0) << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z, msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
}

void PowCore::encoder_callback(const sensor_msgs::msg::JointState::SharedPtr msg){
    z.block<6, 1>(6, 0) << msg->velocity[CR2WR], msg->velocity[CL2WL];
}

void PowCore::right_input_callback(const geometry_msgs::msg::Wrench::SharedPtr msg){
    flag_encoder_r = true;
    u.block<1, 1>(0, 0) << msg->torque.y;
}

void PowCore::left_input_callback(const geometry_msgs::msg::Wrench::SharedPtr msg){
    flag_encoder_l = true;
    u.block<1, 1>(1, 0) << msg->torque.y;
}

void PowCore::joystick_callback(){
    flag_joystick = true;
    theta_dot_d = msg->v_d;
    h_d = msg->h_d;
    phi_d = msg->phi_d;
    yaw_rate_d = msg->ya_rate_d;
}

void PowCore::output_callback(){
    calculate_com_and_inertia();
    get_theta_eq();
    //x_d(0) updated in this moment
}
////////////////////////////////////////////////////////////////////////////
void PowCore::calculate_com_and_inertia(){
    solve_inverse_kinematics();
    solve_forward_kinematics();
    Eigen::Vector3f rB[7];
    for (int i = 0; i < 7; i++) {
        rB[i] = p_vecs.col(i) + R_matrices.at(i) * c_vecs.col(i);
        p_bcom += m_vecs(i) * rB[i];
    }
    p_bcom /= m_B;

    for (int i = 0; i < 7; i++) {
        Eigen::Vector3f rC = rB[i] - p_bcom;
        float nn = rC.squaredNorm();
        I_B_B += R_matrices.at(i) * IG_matrices.at(i) * R_matrices.at(i).transpose() + m_vecs(i) * (nn * Eigen::Matrix3f::Identity() - rC * rC.transpose());
    }
}

void PowCore::solve_inverse_kinematics() {

  // degree to rad
  phi_d = phi_d * M_PI / 180;

  // setting h_saturation
  float phi_max = std::min(atanf((h - HEIGHT_MIN) / L), atanf((HEIGHT_MAX - h) / L));
  float phi_min = -phi_max;

  phi_d = constrain(phi_d, phi_min, phi_max);  // phi값을 제한

  float hR = h - L * tanf(phi_d);
  float hL = h + L * tanf(phi_d);
  float hs[2] = { hR, hL };


  for (int i = 0; i < 2; i++) {
    float h_val = hs[i];

    // 각도 계산

    float angle_ADE = acos((pow(l1, 2) + pow(l4, 2) + pow(l5, 2) - pow(h_val, 2)) / (2 * l1 * sqrt(pow(l4, 2) + pow(l5, 2))));
    float angle_ADC = M_PI - (angle_ADE + angle_EDF);
    float AC = sqrt(pow(l1, 2) + pow(l3, 2) - 2 * l1 * l3 * cos(angle_ADC));
    float S = (pow(AB, 2) + pow(l2, 2) - pow(AC, 2)) / (2 * AB * l2);

    if (abs(S) > 1 && abs(AC - sqrt(pow(a, 2) + pow(b, 2)) - l2) < 0.1) {
      AC = sqrt(pow(a, 2) + pow(b, 2)) + l2;
      S = 1;
    }

    float angle_ABC = acos((pow(AB, 2) + pow(l2, 2) - pow(AC, 2)) / (2 * AB * l2));
    theta_hips[i] = (5 * M_PI / 6) - angle_ABC;
  }

  theta_hips[1] = -theta_hips[1];  // BL 각도 반전
}

void PowCore::solve_forward_kinematics() {
  theta_Bs(0) = -theta_hips(0);
  theta_Bs(1) = theta_hips(1);

  float thB0 = theta_Bs(0), cB0 = cosf(thB0), sB0 = sinf(thB0);
  float thB1 = theta_Bs(1), cB1 = cosf(thB1), sB1 = sinf(thB1);

  float ax0 = a + l2 * cB0, ay0 = b + l2 * sB0;
  float ax1 = a + l2 * cB1, ay1 = b + l2 * sB1;

  float dist0 = sqrtf(ax0 * ax0 + ay0 * ay0);
  float dist1 = sqrtf(ax1 * ax1 + ay1 * ay1);

  float num0 = sqr_fwd + 2 * l2 * (a * cB0 + b * sB0);
  float num1 = sqr_fwd + 2 * l2 * (a * cB1 + b * sB1);

  float r0 = (num0 / (2 * l1)) / dist0;
  float r1 = (num1 / (2 * l1)) / dist1;

  if (fabsf(r0) > 1.0f || fabsf(r1) > 1.0f) {
    throw std::runtime_error("Ratio out of range");
  }

  float alpha0 = atan2f(b + l2 * sB0, a + l2 * cB0);
  float alpha1 = atan2f(b + l2 * sB1, a + l2 * cB1);

  theta_As(0) = -acosf(r0) + alpha0;
  theta_As(1) = -acosf(r1) + alpha1;

  float thA0 = theta_As(0), cA0 = cosf(thA0), sA0 = sinf(thA0);
  float thA1 = theta_As(1), cA1 = cosf(thA1), sA1 = sinf(thA1);

  float nx0 = (a + l2 * cB0 - l1 * cA0), ny0 = (b + l2 * sB0 - l1 * sA0);
  float nx1 = (a + l2 * cB1 - l1 * cA1), ny1 = (b + l2 * sB1 - l1 * sA1);

  theta_ks(0) = atan2f(ny0, nx0);
  theta_ks(1) = atan2f(ny1, nx1);

  // 앞에는 다 상수라서 constructor에서 initialize
  p_vecs.col(5) << -a - l2 * cB0, -0.102f, b + l2 * sB0;
  p_vecs.col(6) << -a - l2 * cB1, 0.102f, b + l2 * sB1;

  R_matrices[1] = rotation_matrix_y(theta_Bs(0));
  R_matrices[2] = rotation_matrix_y(theta_Bs(1));
  R_matrices[3] = rotation_matrix_y(theta_As(0));
  R_matrices[4] = rotation_matrix_y(theta_As(1));
  R_matrices[5] = rotation_matrix_y(theta_ks(0));
  R_matrices[6] = rotation_matrix_y(theta_ks(1));
}

void PowCore::get_theta_eq(){
    theta_eq = atan(-p_bcom(0) / (h + p_bcom(2)));
}

void PowCore::get_theta_hips(){
    // returned theta_hips 
    // publish theta_hips to HR_controller
}

bool PowCore::prepare_state_prediction(){
    prepare_calculate();
    calculate_M();

    if (!calculate_Minv()) {
      return false;
    }
    calculate_nle();
    calculate_dM_dtheta();
    calculate_dnle_dtheta();
    calculate_dnle_dqdot();

    calculate_fx();
    return true;
}

void PowCore::prepare_calculate(){
  cos_theta = cosf(theta);
  sin_theta = sinf(theta);
  cos_theta_2 = cos_theta * cos_theta;
  sin_theta_2 = sin_theta * sin_theta;
  sin_cos_theta = cos_theta * sin_theta;
  h_2 = h * h;
  theta_dot_2 = theta_dot * theta_dot;
  psi_dot_2 = psi_dot * psi_dot;
}

void PowCore::calculate_M(){
    // M(0,0)
    M(0, 0) = I_B_B(1, 1) + m_B * (h_2 + p_bcom(0) * p_bcom(0) + p_bcom(2) * p_bcom(2)) + 2.0f * h * m_B * p_bcom(2);

    // M(1,0) and M(0,1)
    M(1, 0) = m_B * (-sin_theta * p_bcom(0) + cos_theta * (h + p_bcom(2)));
    M(0, 1) = M(1, 0);

    // M(2,0) and M(0,2)
    M(2, 0) = cos_theta * (I_B_B(2, 1) - m_B * p_bcom(1) * (h + p_bcom(2)))
                - sin_theta * (I_B_B(1, 0) - m_B * p_bcom(0) * p_bcom(1));
    M(0, 2) = M(2, 0);

    // M(1,1)
    M(1, 1) = m_B + m_LW + m_RW + 1.0 / (R * R) * (I_B_LW(1, 1) + I_B_RW(1, 1));

    // M(2,1) and M(1,2)
    M(2, 1) = -m_B * p_bcom(1) - L * (m_LW - m_RW) - L * 1.0 / (R * R) * (I_B_LW(1, 1) - I_B_RW(1, 1))
                + (cos_theta * (I_B_LW(2, 1) + I_B_RW(2, 1))) / R
                - (sin_theta * (I_B_LW(1, 0) + I_B_RW(1, 0))) / R;
    M(1, 2) = M(2, 1);

    // M(2,2)
    M(2, 2) = I_B_B(0, 0) + I_B_LW(0, 0) + I_B_RW(0, 0)
                + 1.0 / (R * R) * (L * L * (I_B_LW(1, 1) + I_B_RW(1, 1)))
                + L * L * (m_LW + m_RW)
                + m_B * (h_2 + p_bcom(1) * p_bcom(1) + p_bcom(2) * p_bcom(2))
                - cos_theta * ((I_B_LW(2, 1) * L * 2.0) / R - (I_B_RW(2, 1) * L * 2.0) / R)
                + sin_theta * ((I_B_LW(1, 0) * L * 2.0) / R - (I_B_RW(1, 0) * L * 2.0) / R)
                - cos_theta_2 * (I_B_B(0, 0) - I_B_B(2, 2) + I_B_LW(0, 0) + I_B_RW(0, 0) - I_B_LW(2, 2) - I_B_RW(2, 2) + m_B * (h * p_bcom(2) * 2.0 + h_2 - p_bcom(0) * p_bcom(0) + p_bcom(2) * p_bcom(2)))
                - sin_cos_theta * (I_B_B(2, 0) * 2.0 + I_B_LW(2, 0) * 2.0 + I_B_RW(2, 0) * 2.0 - m_B * (p_bcom(0) * (h + p_bcom(2))) * 2.0)
                + h * m_B * p_bcom(2) * 2.0;
}

bool PowCore::calculate_Minv(){
    M_inv = M.inverse();

    // Check for NaN or Inf values in the result
    if (!M_inv.array().isFinite().all()) {
        std::cout << "[Error] M matrix is Singular matrix." << std::endl;
        return false;
    }
    return true;
}
       
void PowCore::calculate_nle() {
    // First equation (nle(0))
    nle(0) = -psi_dot_2 * (I_B_B(2, 2) + I_B_LW(2, 2) + I_B_RW(2, 2) - m_B * (h * p_bcom(0) + p_bcom(0) * p_bcom(2) - cos_theta_2 * (h * p_bcom(0) * 2.0 + p_bcom(0) * p_bcom(2) * 2.0) - sin_cos_theta * (h * p_bcom(2) * 2.0 + h_2 - p_bcom(0) * p_bcom(0) + p_bcom(2) * p_bcom(2))) - cos_theta_2 * (I_B_B(2, 2) * 2.0 + I_B_LW(2, 2) * 2.0 + I_B_RW(2, 2) * 2.0) + sin_cos_theta * (I_B_B(0, 0) - I_B_B(2, 2) + I_B_LW(0, 0) + I_B_RW(0, 0) - I_B_LW(2, 2) - I_B_RW(2, 2)) + (L * (cos_theta * (I_B_LW(1, 0) - I_B_RW(1, 0)) + sin_theta * (I_B_LW(2, 1) - I_B_RW(2, 1)))) / R)
            - g * m_B * (sin_theta * (h + p_bcom(2)) + p_bcom(0) * cos_theta)
            + (psi_dot * v * (cos_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)) + sin_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)))) / R;

    // Second equation (nle(1))
    nle(1) = -m_B * theta_dot_2 * (sin_theta * (h + p_bcom(2)) + p_bcom(0) * cos_theta)
            - (psi_dot * theta_dot * (cos_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)) + sin_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)))) / R;

    // Third equation (nle(2))
    nle(2) = -theta_dot_2 * (-m_B * (sin_theta * (p_bcom(1) * (h + p_bcom(2))) + p_bcom(0) * p_bcom(1) * cos_theta) + I_B_B(1, 1) * cos_theta + I_B_B(2, 2) * sin_theta)
            + psi_dot * theta_dot * (I_B_B(2, 2) * 2.0 + I_B_LW(2, 2) * 2.0 + I_B_RW(2, 2) * 2.0 - m_B * (h * p_bcom(0) * 2.0 + p_bcom(0) * p_bcom(2) * 2.0 - cos_theta_2 * (p_bcom(0) * 4.0 * (h + p_bcom(2))) - sin_cos_theta * (h * p_bcom(2) * 4.0 + h_2 * 2.0 - p_bcom(0) * p_bcom(0) * 2.0 + p_bcom(2) * p_bcom(2) * 2.0)))
            - cos_theta_2 * (I_B_B(2, 2) * 4.0 + I_B_LW(2, 2) * 4.0 + I_B_RW(2, 2) * 4.0)
            + sin_cos_theta * (I_B_B(0, 0) * 2.0 - I_B_B(2, 2) * 2.0 + I_B_LW(0, 0) * 2.0 + I_B_RW(0, 0) * 2.0 - I_B_LW(2, 2) * 2.0 - I_B_RW(2, 2) * 2.0)
            + (L * (cos_theta * (I_B_LW(1, 0) * 2.0 - I_B_RW(1, 0) * 2.0) + sin_theta * (I_B_LW(2, 1) * 2.0 - I_B_RW(2, 1) * 2.0))) / R
            - (theta_dot * v * (cos_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)) + sin_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)))) / R;
}

void PowCore::calculate_dM_dtheta(){
    // Compute the elements of the matrix
    dM_dtheta(1, 0) = -m_B * p_bcom(0) * cos_theta - m_B * sin_theta * (h + p_bcom(2));
    dM_dtheta(2, 0) = -cos_theta * (I_B_B(1, 0) - m_B * p_bcom(0) * p_bcom(1))
                        - sin_theta * (I_B_B(2, 0) - m_B * (p_bcom(1) * (h + p_bcom(2))));

    dM_dtheta(0, 1) = dM_dtheta(1, 0);
    dM_dtheta(1, 1) = 0.0;
    dM_dtheta(2, 1) = -(cos_theta * (I_B_LW(1, 0) + I_B_RW(1, 0))) / R
                        - (sin_theta * (I_B_LW(2, 0) + I_B_RW(2, 0))) / R;

    dM_dtheta(0, 2) = dM_dtheta(2, 0);
    dM_dtheta(1, 2) = dM_dtheta(2, 1);

    dM_dtheta(2, 2) = I_B_B(2, 0) * 2.0 + I_B_LW(2, 0) * 2.0 + I_B_RW(2, 0) * 2.0
                        - cos_theta_2 * (I_B_B(2, 0) * 4.0 + I_B_LW(2, 0) * 4.0 + I_B_RW(2, 0) * 4.0 - m_B * p_bcom(0) * 4.0 * (h + p_bcom(2)))
                        - m_B * (p_bcom(0) * 2.0 * (h + p_bcom(2)))
                        + cos_theta * ((I_B_LW(1, 0) * L * 2.0) / R - (I_B_RW(1, 0) * L * 2.0) / R)
                        + sin_theta * ((I_B_LW(2, 0) * L * 2.0) / R - (I_B_RW(2, 0) * L * 2.0) / R)
                        + sin_cos_theta * (I_B_B(0, 0) * 2.0 - I_B_B(2, 0) * 2.0 + I_B_LW(0, 0) * 2.0 + I_B_RW(0, 0) * 2.0 - I_B_LW(2, 0) * 2.0 - I_B_RW(2, 0) * 2.0 + m_B * (h * p_bcom(2) * 4.0 - p_bcom(0) * p_bcom(0) * 2.0) + m_B * (h_2 + p_bcom(2) * p_bcom(2)) * 2.0);
}

void PowCore::calculate_dnle_dtheta(){
    dnle_dtheta(0) = -psi_dot_2 * (-I_B_B(0, 0) + I_B_B(2, 2) - I_B_LW(0, 0) - I_B_RW(0, 0) + I_B_LW(2, 2) + I_B_RW(2, 2) - m_B * (2.0 * h * p_bcom(2) - 2.0 * cos_theta_2 * (h * p_bcom(2) * 2.0 + pow(h, 2) - pow(p_bcom(0), 2) + pow(p_bcom(2), 2)) + pow(h, 2) - pow(p_bcom(0), 2) + pow(p_bcom(2), 2) + 4.0 * sin_cos_theta * p_bcom(0) * (h + p_bcom(2))) + 2.0 * cos_theta_2 * (I_B_B(0, 0) - I_B_B(2, 2) + I_B_LW(0, 0) + I_B_RW(0, 0) - I_B_LW(2, 2) - I_B_RW(2, 2)) + 4.0 * sin_cos_theta * (I_B_B(2, 0) + I_B_LW(2, 0) + I_B_RW(2, 0)) + (L * (cos_theta * (I_B_LW(2, 1) - I_B_RW(2, 1)) - sin_theta * (I_B_LW(1, 0) - I_B_RW(1, 0)))) / R)
                   - g * m_B * (cos_theta * (h + p_bcom(2)) - p_bcom(0) * sin_theta)
                   + (psi_dot * v * (cos_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)) - sin_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)))) / R;

    dnle_dtheta(1) = -m_B * theta_dot_2 * (cos_theta * (h + p_bcom(2)) - p_bcom(0) * sin_theta)
                    - (psi_dot * theta_dot * (cos_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)) - sin_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)))) / R;

    dnle_dtheta(2) = theta_dot_2 * (m_B * (cos_theta * (h * p_bcom(1) + p_bcom(1) * p_bcom(2)) - p_bcom(0) * p_bcom(1) * sin_theta) - I_B_B(2, 1) * cos_theta + I_B_B(1, 0) * sin_theta)
                    + psi_dot * theta_dot * (2.0 * (-I_B_B(0, 0) + I_B_B(2, 2) - I_B_LW(0, 0) - I_B_RW(0, 0) + I_B_LW(2, 2) + I_B_RW(2, 2)) - m_B * (4.0 * h * p_bcom(2) - 4.0 * cos_theta_2 * (h * p_bcom(2) * 2.0 + pow(h, 2) - pow(p_bcom(0), 2) + pow(p_bcom(2), 2)) + 2.0 * (pow(h, 2) - pow(p_bcom(0), 2) + pow(p_bcom(2), 2)) + 8.0 * sin_cos_theta * (h * p_bcom(0) + p_bcom(0) * p_bcom(2))) + 4.0 * cos_theta_2 * (I_B_B(0, 0) - I_B_B(2, 2) + I_B_LW(0, 0) + I_B_RW(0, 0) - I_B_LW(2, 2) - I_B_RW(2, 2)) + 8.0 * sin_cos_theta * (I_B_B(2, 0) + I_B_LW(2, 0) + I_B_RW(2, 0)) + (L * (2.0 * cos_theta * (I_B_LW(2, 1) - I_B_RW(2, 1)) - 2.0 * sin_theta * (I_B_LW(1, 0) - I_B_RW(1, 0)))) / R)
                    - (theta_dot * v * (cos_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)) - sin_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)))) / R;
}

void PowCore::calculate_dnle_dqdot(){
    dnle_dqdot(0, 0) = 0.0;

    dnle_dqdot(1, 0) = -psi_dot * (cos_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)) + sin_theta * (I_B_LW(2, 1) + I_B_RW(2, 1))) / R
                        - 2.0f * m_B * theta_dot * (sin_theta * (h + p_bcom(2)) + p_bcom(0) * cos_theta);

    dnle_dqdot(2, 0) = psi_dot * (2.0f * (I_B_B(2, 0) + I_B_LW(2, 0) + I_B_RW(2, 0)) - m_B * (2.0f * (h * p_bcom(0) + p_bcom(0) * p_bcom(2)) - 4.0f * cos_theta_2 * (h * p_bcom(0) + p_bcom(0) * p_bcom(2)) - sin_cos_theta * (4.0f * h * p_bcom(2) + 2.0f * h_2 - 2.0f * p_bcom(0) * p_bcom(0) + 2.0f * p_bcom(2) * p_bcom(2))) - 4.0f * cos_theta_2 * (I_B_B(2, 0) + I_B_LW(2, 0) + I_B_RW(2, 0)) + 2.0f * sin_cos_theta * (I_B_B(0, 0) - I_B_B(2, 2) + I_B_LW(0, 0) + I_B_RW(0, 0) - I_B_LW(2, 2) - I_B_RW(2, 2)) + (L * (cos_theta * (2.0f * I_B_LW(1, 0) - 2.0f * I_B_RW(1, 0)) + sin_theta * (2.0f * I_B_LW(2, 1) - 2.0f * I_B_RW(2, 1)))) / R)
                        - theta_dot * (-m_B * (2.0f * sin_theta * (h * p_bcom(1) + p_bcom(1) * p_bcom(2)) + 2.0f * p_bcom(0) * p_bcom(1) * cos_theta) + 2.0f * I_B_B(1, 0) * cos_theta + 2.0f * I_B_B(2, 1) * sin_theta)
                        - (v * (cos_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)) + sin_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)))) / R;

    dnle_dqdot(0, 1) = (psi_dot * (cos_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)) + sin_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)))) / R;

    dnle_dqdot(1, 1) = 0.0;

    dnle_dqdot(2, 1) = -(theta_dot * (cos_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)) + sin_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)))) / R;

    dnle_dqdot(0, 2) = -psi_dot * (2.0f * (I_B_B(2, 0) + I_B_LW(2, 0) + I_B_RW(2, 0)) - m_B * (2.0f * (h * p_bcom(0) + p_bcom(0) * p_bcom(2)) - cos_theta_2 * (4.0f * h * p_bcom(0) + 4.0f * p_bcom(0) * p_bcom(2)) - sin_cos_theta * (4.0f * h * p_bcom(2) + 2.0f * h_2 - 2.0f * p_bcom(0) * p_bcom(0) + 2.0f * p_bcom(2) * p_bcom(2))) - 4.0f * cos_theta_2 * (I_B_B(2, 0) + I_B_LW(2, 0) + I_B_RW(2, 0)) + 2.0f * sin_cos_theta * (I_B_B(0, 0) - I_B_B(2, 2) + I_B_LW(0, 0) + I_B_RW(0, 0) - I_B_LW(2, 2) - I_B_RW(2, 2)) + (L * (cos_theta * (2.0f * I_B_LW(1, 0) - 2.0f * I_B_RW(1, 0)) + sin_theta * (2.0f * I_B_LW(2, 1) - 2.0f * I_B_RW(2, 1)))) / R)
                        + (v * (cos_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)) + sin_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)))) / R;

    dnle_dqdot(1, 2) = -(theta_dot * (cos_theta * (I_B_LW(1, 0) + I_B_RW(1, 0)) + sin_theta * (I_B_LW(2, 1) + I_B_RW(2, 1)))) / R;

    dnle_dqdot(2, 2) = theta_dot * (2.0f * (I_B_B(2, 0) + I_B_LW(2, 0) + I_B_RW(2, 0)) - m_B * (2.0f * (h * p_bcom(0) + p_bcom(0) * p_bcom(2)) - cos_theta_2 * (4.0f * h * p_bcom(0) + 4.0f * p_bcom(0) * p_bcom(2)) - sin_cos_theta * (4.0f * h * p_bcom(2) + 2.0f * h_2 - 2.0f * p_bcom(0) * p_bcom(0) + 2.0f * p_bcom(2) * p_bcom(2))) - 4.0f * cos_theta_2 * (I_B_B(2, 0) + I_B_LW(2, 0) + I_B_RW(2, 0)) + 2.0f * sin_cos_theta * (I_B_B(0, 0) - I_B_B(2, 2) + I_B_LW(0, 0) + I_B_RW(0, 0) - I_B_LW(2, 2) - I_B_RW(2, 2)) + (L * (cos_theta * (2.0f * I_B_LW(1, 0) - 2.0f * I_B_RW(1, 0)) + sin_theta * (2.0f * I_B_LW(2, 1) - 2.0f * I_B_RW(2, 1)))) / R);
}
       
void PowCore::calculate_fx(){
    fx.block<3, 1>(1, 0) = -M_inv * (dM_dtheta * M_inv * (-nle + B * u) + (dnle_dtheta));
    fx.block<3, 3>(1, 1) = -M_inv * dnle_dqdot;
}

bool PowCore::estimate_state() {
    if (!predict()) {
      return false;
    }
    update();
    x_ = x;
    return true;
}

bool PowCore::predict() {
    if (!prepare_state_prediction()) {
        return false;
    }  // update M, nle, dM_dtheta, dnle_dtheta, dnle_dqdot
    // 예측 단계 수행
    predict_state(x_pred, F_mat);
    P_pred = F_mat * P * F_mat.transpose() + Q_cov.toDenseMatrix();
    return true;
}

void PowCore::update() {
    predict_measurement();                                                                        // h_obs 및 H 업데이트
    K_mat = P_pred * H.transpose() * (H * P_pred * H.transpose() + R_cov.toDenseMatrix()).inverse();  // 칼만 이득 계산
    x = x_pred + K_mat * (z - h_obs);                                                                 // 상태 업데이트
    P = (Eigen::Matrix<float, 4, 4>::Identity() - K_mat * H) * P_pred;                                // 오차 공분산 업데이트
}

void PowCore::predict_state() {
    f(0) = x(1);
    f.segment<3>(1) = M_inv * (-nle + B * u);
    x_pred = x + f * dt;
    F_mat = fx;
}

void PowCore::predict_measurement() {
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

    float h = Pol_ref.h;
    float g = Pol_ref.g;
    float L = Pol_ref.L;
    float R = Pol_ref.R;



    // h_obs 계산
    h_obs(0) = h * theta_ddot + v_dot * cos_theta - g * sin_theta - h * std::pow(psi_dot, 2) * sin_cos_theta;
    h_obs(1) = psi_dot * v + h * psi_ddot * sin_theta + h * psi_dot * theta_dot * cos_theta * 2.0;
    h_obs(2) = -h * std::pow(theta_dot, 2) + g * cos_theta + v_dot * sin_theta - std::pow(psi_dot, 2) * (h - h * cos_theta_2);
    h_obs(3) = -psi_dot * sin_theta;
    h_obs(4) = theta_dot;
    h_obs(5) = psi_dot * cos_theta;
    h_obs(6) = theta_dot - v / R - (L * psi_dot) / R;
    h_obs(7) = -theta_dot + v / R - (L * psi_dot) / R;

    // H 행렬 계산
    H(0, 0) = std::pow(psi_dot, 2) * (h * sin_theta_2 - h * cos_theta_2) - g * cos_theta - v_dot * sin_theta;
    H(1, 0) = h * psi_ddot * cos_theta - h * psi_dot * theta_dot * sin_theta * 2.0;
    H(2, 0) = v_dot * cos_theta - g * sin_theta - h * std::pow(psi_dot, 2) * sin_cos_theta * 2.0;
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


inline Eigen::Matrix<float, 3, 3> PowCore::rotation_matrix_y(float theta) {
  float c = cosf(theta), s = sinf(theta);
  Eigen::Matrix<float, 3, 3> R;
  R << c, 0, s, 0, 1, 0, -s, 0, c;
  return R;
}

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PowCore>());
    rclcpp::shutdown();
    return 0;
}