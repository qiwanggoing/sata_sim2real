#include <deploy_real/low_level_ctrl.hpp>

LowLevelControl::LowLevelControl() : Node("low_level_control_node")
{

    state_suber_ = this->create_subscription<unitree_go::msg::LowState>(
        "/lowstate", 10, std::bind(&LowLevelControl::state_callback, this, std::placeholders::_1));
}

void LowLevelControl::state_callback(unitree_go::msg::LowState::SharedPtr msg)
{
    for (int i = 0; i < 12; i++)
        {motor[i] = msg->motor_state[i];
        q_init_[i] = motor[i].q;}

    // motion_time_++;
    // first, get record initial position
    // if (motion_time_ >= 0 && motion_time_ < 20)

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);                        // Initialize rclcpp
    auto node = std::make_shared<LowLevelControl>(); // Create a ROS2 node and make share with low_level_cmd_sender class
    rclcpp::spin(node);                              // Run ROS2 node
    rclcpp::shutdown();                              // Exit
    return 0;
}