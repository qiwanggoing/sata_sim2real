
#include <deploy_real/low_level_ctrl.hpp>

float clip_val(float val, int index)
{
    if ((index + 1) % 3 == 0)
    {
        if (val > 35.55)
            val = 35.55;
        else if (val < -35.55)
            val = -35.55;
    }
    else
    {
        if (val > 23.7)
            val = 23.7;
        else if (val < -23.7)
            val = -23.7;
    }
    return val;
}

double get_diff_norm(vector<double> &vec1, vector<double> &vec2)
{
    assert(vec1.size() == vec2.size());
    double sum = 0.0;
    int n = vec1.size();
    for (int i = 0; i < n; i++)
    {
        sum += pow((vec1[i] - vec2[i]), 2);
    }
    return sqrt(sum);
}

double get_norm(vector<double> &vec)
{
    double sum = 0.0;
    for (double num : vec)
        sum += pow(num, 2);
    return sqrt(sum);
}

LowLevelControl::LowLevelControl() : Node("low_level_control_node")
{
    init_cmd();
    target_torque_puber_ = this->create_publisher<unitree_go::msg::LowCmd>("/mujoco/lowcmd", 10);
    state_suber_ = this->create_subscription<unitree_go::msg::LowState>(
        "/mujoco/lowstate", 10, std::bind(&LowLevelControl::state_callback, this, std::placeholders::_1));
    target_pos_suber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/rl/target_pos", 10, std::bind(&LowLevelControl::target_pos_callback, this, std::placeholders::_1));
    joy_suber_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, bind(&LowLevelControl::joy_callback, this, placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(5), std::bind(&LowLevelControl::timer_callback, this));
}

void LowLevelControl::init_cmd()
{
    kp = vector<double>(12, 20.0);
    kd = vector<double>(12, 1);
    for (int i = 0; i < 20; i++)
    {
        cmd_msg_.motor_cmd[i].mode = 0x01; // Set toque mode, 0x00 is passive mode
        cmd_msg_.motor_cmd[i].q = PosStopF;
        cmd_msg_.motor_cmd[i].kp = 0;
        cmd_msg_.motor_cmd[i].dq = VelStopF;
        cmd_msg_.motor_cmd[i].kd = 0;
        cmd_msg_.motor_cmd[i].tau = 0;
    }
}

void LowLevelControl::state_callback(unitree_go::msg::LowState::SharedPtr msg)
{
    for (int i = 0; i < 12; i++)
    {
        motor[i] = msg->motor_state[i];
    }
}

void LowLevelControl::target_pos_callback(std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    recieved_data_ = true;
    rl_target_pos_.data = msg->data;
}

void LowLevelControl::joy_callback(sensor_msgs::msg::Joy::SharedPtr msg)
{

    if (is_laydown_ and msg->buttons[1]) // Button B
    {
        should_stand_ = true;
        should_laydown_ = false;
        should_run_policy_ = false;
    }
    else if (is_standing_ and msg->buttons[0]) // Button A
    {
        should_laydown_ = true;
        should_stand_ = false;
        should_run_policy_ = false;
    }
    else if (is_standing_ and msg->buttons[4] and msg->buttons[5]) // BUtton LB and Button RB
    {
        should_laydown_ = false;
        should_stand_ = false;
        should_run_policy_ = true;
    }
}

void LowLevelControl::timer_callback()
{
    state_obs();
    if (is_uncontrolled_ and !is_laydown_) // init pos: lay down
    {
        state_transform(laydown_angels_);
        cout << "should init" << endl;
    }
    else if (is_laydown_ and should_stand_) // from lay down state to stand up state
    {
        state_transform(standing_angels_);
        cout << "should stand up" << endl;
    }
    else if (is_standing_ and should_laydown_) // from stand up state to lay down state
    {
        state_transform(laydown_angels_);
        cout << "should lay down" << endl;
    }
    else if (is_standing_ and should_run_policy_) // from stand up state to policy state
    {
        run_policy();
        cout << "should run policy" << endl;
    }
    else // keep last action
    {
        vector<double> vec(12, 0);
        vector<double> target_angles;
        if (is_standing_)
            target_angles = standing_angels_;
        else
            target_angles = laydown_angels_;
        for (int i = 0; i < 12; i++)
        {
            cmd_msg_.motor_cmd[i].mode = 0x01; // Set toque mode, 0x00 is passive mode
            cmd_msg_.motor_cmd[i].q = 0;
            cmd_msg_.motor_cmd[i].kp = 0;
            cmd_msg_.motor_cmd[i].dq = 0;
            cmd_msg_.motor_cmd[i].kd = 0;
            cmd_msg_.motor_cmd[i].tau = clip_val(50 * (target_angles[i] - motor[i].q) + 1 * (0 - motor[i].dq),i);
        }
        get_crc(cmd_msg_);
        target_torque_puber_->publish(cmd_msg_);
        cout << "should keep" << endl;
    }
}
void LowLevelControl::run_policy()
{
    if (!recieved_data_)
    {
        cout << "Have not recieved RL data yet" << endl;
        target_torque_puber_->publish(cmd_msg_);
        return;
    }
    // for (int i = 0; i < 12; i++)
    // {
    //     cmd_msg_.motor_cmd[i].mode = 0x01; // Set toque mode, 0x00 is passive mode
    //     cmd_msg_.motor_cmd[i].q = 0;
    //     cmd_msg_.motor_cmd[i].kp = 0;
    //     cmd_msg_.motor_cmd[i].dq = 0;
    //     cmd_msg_.motor_cmd[i].kd = 0;
    //     cmd_msg_.motor_cmd[i].tau = clip_val(20 * (rl_target_pos_.data[i] - motor[i].q) + 0.5 * (0 - motor[i].dq),i);
    // }
            for (int i = 0; i < 12; i++)
    {
        cmd_msg_.motor_cmd[i].mode = 0x01; // Set toque mode, 0x00 is passive mode
        cmd_msg_.motor_cmd[i].q = standing_angels_[i];
        cmd_msg_.motor_cmd[i].kp = kp[i];
        cmd_msg_.motor_cmd[i].dq = 0;
        cmd_msg_.motor_cmd[i].kd = kd[i];
        cmd_msg_.motor_cmd[i].tau = 0;
        double val= 20 * (rl_target_pos_.data[i] - motor[i].q) + 0.5 * (0 - motor[i].dq);
        double clipped_val=clip_val(val,i);
        cout<<"original tau"<<i<<"value:"<<val<<endl;
        cout<<"clipped tau"<<i<<"value:"<<clipped_val<<endl;

    }
    get_crc(cmd_msg_);
    target_torque_puber_->publish(cmd_msg_);
    cout << endl;
}
void LowLevelControl::state_obs()
{
    vector<double> q(12, 0);
    vector<double> dq(12, 0);

    for (int i = 0; i < 12; i++)
    {
        q[i] = motor[i].q;
        dq[i] = motor[i].dq;
    }
    cout << "diff norm" << get_diff_norm(q, laydown_angels_) << endl;
    cout << "dq norm" << get_norm(dq) << endl;
    cout << "standing diff norm" << get_diff_norm(q, standing_angels_) << endl;
    if (get_diff_norm(q, laydown_angels_) < 0.25 && get_norm(dq) < 0.15)
    {
        if (!is_laydown_)
        {
            motion_time_ = 0;
            rate_count_ = 0;
        }
        is_laydown_ = true;
        is_uncontrolled_ = false;
        is_standing_ = false;
    }

    else if (get_diff_norm(q, standing_angels_) < 0.33 && get_norm(dq) < 0.15)
    {
        if (!is_standing_)
        {
            motion_time_ = 0;
            rate_count_ = 0;
        }
        is_standing_ = true;
        is_laydown_ = false;
        is_uncontrolled_ = false;
    }
}
void LowLevelControl::state_transform(vector<double> &target_angels)
{
    motion_time_++;
    // first, get record initial position
    if (motion_time_ >= 0 && motion_time_ < 20)
        for (int i = 0; i < 12; i++)
            q_init_[i] = motor[i].q;
    // second, move to the origin point of a sine movement with Kp Kd
    if (motion_time_ >= 20)
    {
        rate_count_++;

        double rate = rate_count_ / 400.0; // needs count to 200
        vector<double> vec(12, 0);
        for (int i = 0; i < 12; i++)
        {
            q_des_[i] = jointLinearInterpolation(q_init_[i], target_angels[i], rate);
            cmd_msg_.motor_cmd[i].mode = 0x01; // Set toque mode, 0x00 is passive mode
            cmd_msg_.motor_cmd[i].q = 0;
            cmd_msg_.motor_cmd[i].kp = 0;
            cmd_msg_.motor_cmd[i].dq = 0;
            cmd_msg_.motor_cmd[i].kd = 0;
            cmd_msg_.motor_cmd[i].tau = clip_val(50 * (q_des_[i] - motor[i].q) + 1 * (0 - motor[i].dq),i);
        }
        get_crc(cmd_msg_);
        target_torque_puber_->publish(cmd_msg_);
    }
}

double LowLevelControl::jointLinearInterpolation(double initPos, double targetPos, double rate)
{
    double p;
    rate = std::min(std::max(rate, 0.0), 1.0);
    p = initPos * (1 - rate) + targetPos * rate;
    return p;
}

// LowLevelControl::~LowLevelControl()
// {
// }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                        // Initialize rclcpp
    auto node = std::make_shared<LowLevelControl>(); // Create a ROS2 node and make share with low_level_cmd_sender class

    rclcpp::spin(node); // Run ROS2 node
    rclcpp::shutdown(); // Exit
    return 0;
}
