#include "base_velocity_estimator/ekf_velocity_estimator.hpp"

// EKFVelocityEstimator::EKFVelocityEstimator()
//     : Node("base_velocity_estimator")
// {   
//     Q_.block<3,3>(3,3) *= 1e-5;  
//     Q_.block<3,3>(6,6) *= 0.001;  

//     bias_calibration_samples = 1000;
//     H_(0, 0) = 1.0; // vx
//     H_(1, 1) = 1.0; // vy
//     H_(2, 2) = 1.0; // vz
//     velocity_base_ = Eigen::Vector3d::Zero();
//     accel_bias_ = Eigen::Vector3d::Zero();
//     is_bias_calibrated_ = false;
    
//     imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
//         "/imu/data", 10, std::bind(&EKFVelocityEstimator::imuCallback, this, std::placeholders::_1));
//     contact_sub = create_subscription<std_msgs::msg::Float32MultiArray>(
//         "/contact", 10, std::bind(&EKFVelocityEstimator::contactDetector, this, std::placeholders::_1));

//     velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>(
//         "/base_velocity", 10);
//     vel_obs_sub_ = create_subscription<geometry_msgs::msg::Twist>("/obs_velocity", 10,
//                                                                   std::bind(&EKFVelocityEstimator::obsCallback, this, std::placeholders::_1));
//     double publish_rate = 50;

//     timer_ = create_wall_timer(
//         std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
//         std::bind(&EKFVelocityEstimator::publishVelocity, this));
//     RCLCPP_INFO(get_logger(), "Node initialized. Waiting for IMU data...");
// }
EKFVelocityEstimator::EKFVelocityEstimator()
    : Node("base_velocity_estimator")
{   
    // ... (保留你的 Q, R, H 初始化代码) ...
    Q_.block<3,3>(3,3) *= 1e-5;  
    Q_.block<3,3>(6,6) *= 0.001;  
    bias_calibration_samples = 1000;
    H_.setIdentity(); 

    velocity_base_ = Eigen::Vector3d::Zero();
    accel_bias_ = Eigen::Vector3d::Zero();
    is_bias_calibrated_ = false;

    // 1. 修改订阅逻辑: 订阅 /lowstate
    lowstate_sub_ = create_subscription<unitree_go::msg::LowState>(
        "lowstate", rclcpp::SensorDataQoS(), 
        std::bind(&EKFVelocityEstimator::lowStateCallback, this, std::placeholders::_1));
    // 保留你的 velocity publisher (注意：Sim2Real时 rl_policy 需要订阅这个话题)
    velocity_pub_ = create_publisher<geometry_msgs::msg::Twist>("/ekf/velocity", 10);
    
    // 保留观测速度订阅 (来自运动学)
    vel_obs_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/obs_velocity", 10,
        std::bind(&EKFVelocityEstimator::obsCallback, this, std::placeholders::_1));

    // 保持原来的 Timer 或直接在回调中发布
    double publish_rate = 200; // 建议提高到 200Hz 配合 lowstate
    timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
        std::bind(&EKFVelocityEstimator::publishVelocity, this));
        
    RCLCPP_INFO(get_logger(), "Sim2Real EKF Node initialized. Listening to /lowstate...");
}
// void EKFVelocityEstimator::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
// {
//     auto current_time = rclcpp::Time(msg->header.stamp);

//     Eigen::Vector3d accel_measured = {
//         msg->linear_acceleration.x,
//         msg->linear_acceleration.y,
//         msg->linear_acceleration.z};
//     if (!is_bias_calibrated_)
//     {
//         accel_samples_.push_back(accel_measured);
//         if (accel_samples_.size() >= bias_calibration_samples)
//         {
//             calibrateBias();
//             is_bias_calibrated_ = true;
//             RCLCPP_INFO(get_logger(), "Bias calibration complete. Bias: [%f, %f, %f]",
//                         accel_bias_.x(), accel_bias_.y(), accel_bias_.z());
//         }
//         return;
//     }
//     if (last_time_.nanoseconds() == 0)
//     {
//         last_time_ = current_time;
//         return;
//     }
//     double dt = (current_time - last_time_).seconds();
//     last_time_ = current_time;

//     Eigen::Vector3d accel_corrected = accel_measured - accel_bias_;
//     // velocity_base_ += accel_corrected * dt;
//     predict(accel_corrected, dt);
//     accel_corrected.norm() > 0.1 ? small_acc_ = false : small_acc_ = true;
// }
// 2. 全新的回调函数 (替换原来的 imuCallback)
void EKFVelocityEstimator::lowStateCallback(const unitree_go::msg::LowState::SharedPtr msg)
{
    // 获取当前时间
    auto current_time = this->now();

    // A. 解析 IMU 加速度 (注意单位，Unitree通常是 m/s^2)
    Eigen::Vector3d accel_measured(
        msg->imu_state.accelerometer[0],
        msg->imu_state.accelerometer[1],
        msg->imu_state.accelerometer[2]
    );

    // B. 解析 IMU 四元数 (注意顺序: w, x, y, z)
    Eigen::Quaterniond q(
        msg->imu_state.quaternion[0], // w
        msg->imu_state.quaternion[1], // x
        msg->imu_state.quaternion[2], // y
        msg->imu_state.quaternion[3]  // z
    );

    // C. 静态零偏校准 (注意：这里的 Bias 不应包含重力)
    if (!is_bias_calibrated_) {
        accel_samples_.push_back(accel_measured);
        if (accel_samples_.size() >= bias_calibration_samples) {
            calibrateBias(); 
            // 修正: 你的 calibrateBias 算的是均值。
            // 如果校准时机器人是水平静止的，均值里包含了 [0,0,9.81]。
            // 我们需要把重力剥离出去，剩下的才是传感器误差。
            Eigen::Vector3d gravity_vertical(0, 0, 9.81);
            accel_bias_ = accel_bias_ - gravity_vertical; 
            
            is_bias_calibrated_ = true;
            RCLCPP_INFO(get_logger(), "Bias calibrated (Gravity Removed): [%f, %f, %f]",
                        accel_bias_.x(), accel_bias_.y(), accel_bias_.z());
        }
        return;
    }

    // D. 计算 dt
    if (last_time_.nanoseconds() == 0) {
        last_time_ = current_time;
        return;
    }
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    // 保护 dt 防止积分爆炸
    if(dt > 0.05 || dt < 0.0001) dt = 0.005; 

    // E. !!! 核心物理计算: 动态去除重力 !!!
    // 机器狗在运动中会倾斜，所以不能简单减去 [0,0,9.81]
    // 必须计算：重力在当前机身坐标系下的分量
    Eigen::Vector3d gravity_world(0, 0, 9.81);
    Eigen::Vector3d gravity_body = q.inverse() * gravity_world; 

    // 纯运动加速度 = 测量值 - 传感器零偏 - 重力分量
    Eigen::Vector3d accel_motion = accel_measured - accel_bias_ - gravity_body;

    // F. 执行 EKF 预测
    predict(accel_motion, dt);

    // G. (可选) 利用足端压力进行零速修正 (ZUPT)
    // 如果四只脚压力都很大且加速度很小，说明可能在站立
    // float fl = msg->foot_force[0]; ...
    // 你的原代码里有 contactDetector，可以适配过来
}

void EKFVelocityEstimator::contactDetector(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    float FL_force = msg->data[0];
    float FR_force = msg->data[1];
    float RL_force = msg->data[2];
    float RR_force = msg->data[3];
    updateForceBuffer(fl_force_buffer_, FL_force);
    updateForceBuffer(fr_force_buffer_, FR_force);
    updateForceBuffer(rl_force_buffer_, RL_force);
    updateForceBuffer(rr_force_buffer_, RR_force);
    bool legs_stable = isForceStable(fl_force_buffer_) && isForceStable(fr_force_buffer_) && isForceStable(rr_force_buffer_) && isForceStable(rl_force_buffer_);
    // RCLCPP_INFO(get_logger(), "FL: %.2f, FR: %.2f, RL: %.2f, RR: %.2f",
    // FL_force, FR_force, RL_force, RR_force);
    if (small_acc_ && legs_stable)
    {
        x_(0) = 0;
        x_(1) = 0;
        x_(2) = 0;
    }
}

void EKFVelocityEstimator::updateForceBuffer(std::deque<float> &force_buffer, float new_value)
{
    force_buffer.push_back(new_value);
    if (force_buffer.size() > window_size_)
        force_buffer.pop_front();
}
bool EKFVelocityEstimator::isForceStable(std::deque<float> &force_buffer)
{

    if (force_buffer.size() < window_size_)
        return false;
    float sq_sum = 0.0f;
    for (auto x : force_buffer)
    {
        sq_sum += x * x;
    }
    float mean = std::accumulate(force_buffer.begin(), force_buffer.end(), 0.0f) / window_size_;
    float stddev = std::sqrt(sq_sum / window_size_ - mean * mean);
    return stddev < stable_threshold_stddev;
}
void EKFVelocityEstimator::calibrateBias()
{
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const auto &sample : accel_samples_)
    {
        sum += sample;
    }
    accel_bias_ = sum / static_cast<double>(accel_samples_.size());
    accel_samples_.clear();
}

void EKFVelocityEstimator::predict(const Eigen::Vector3d &imu_acc, double dt)
{

    double b_ax = x_(6);
    double b_ay = x_(7);
    double b_az = x_(8);
    double ax = imu_acc(0) - b_ax;
    double ay = imu_acc(1) - b_ay;
    double az = imu_acc(2) - b_az;
    x_(0) += ax * dt;
    x_(1) += ay * dt;
    x_(2) += az * dt;

    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(9, 9);
    F(0, 3) = dt;  // vx to ax
    F(0, 6) = -dt; // vx to bax
    F(1, 4) = dt;  // vy to ay
    F(1, 7) = -dt; // vy to bay
    F(2, 5) = dt;  // vz to az
    F(2, 8) = -dt; // vz to baz
    P_ = F * P_ * F.transpose() + Q_;
}
void EKFVelocityEstimator::update(const Eigen::Vector3d &vel_obs)
{
    Eigen::Vector3d z = vel_obs;
    Eigen::Vector3d y = z - H_ * x_;

    Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_;

    Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

    x_ += K * y;
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(9, 9);
    P_ = (I - K * H_) * P_;
}
void EKFVelocityEstimator::publishVelocity()
{
    if (!is_bias_calibrated_)
        return;

    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = x_(0);
    msg.linear.y = x_(1);
    msg.linear.z = x_(2);
    velocity_pub_->publish(msg);
}
void EKFVelocityEstimator::obsCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    Eigen::Vector3d obs_vel;
    obs_vel(0) = msg->linear.x;
    obs_vel(1) = msg->linear.y;
    obs_vel(2) = msg->linear.z;
    update(obs_vel);
}

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<EKFVelocityEstimator>();
//     rclcpp::spin(node);

//     rclcpp::shutdown();
//     return 0;
// }