#!/usr/bin/env python3
import rclpy
import torch
# 绝对导入
from sata_config import Go2Config 
from sata_policy_loader import RLPolicy 

import numpy as np
import os
import sys
from rclpy.node import Node
import argparse
from pathlib import Path
from unitree_go.msg import LowState
from std_msgs.msg import Float32MultiArray
# 绝对导入
from xbox_command import XboxController 
# !!! 新增导入 !!!
from geometry_msgs.msg import TwistStamped 

project_root=Path(__file__).parents[4]

class dataReciever(Node):
    def __init__(self,config:Go2Config, policy: RLPolicy, simulation: bool):
        super().__init__("data_reciever")
        self.config = config
        self.policy = policy
        self.cmd_sub=XboxController(self)
        
        # ... (SATA 状态变量初始化不变) ...
        self.num_actions = 12
        self.last_torques = np.zeros(self.num_actions, dtype=np.float32)
        self.motor_fatigue = np.zeros(self.num_actions, dtype=np.float32)
        self.activation_sign = np.zeros(self.num_actions, dtype=np.float32)
        self.dof_vel_limits = np.full(self.num_actions, self.config.DOF_VEL_LIMITS, dtype=np.float32)
        self.torque_limits = np.full(self.num_actions, self.config.TORQUE_LIMITS, dtype=np.float32)
        self.GAMMA = self.config.GAMMA 
        self.BETA = self.config.BETA
        self.ACTION_SCALE = self.config.ACTION_SCALE
        self.CMD_SCALES = np.array([
            self.config.COMMANDS_SCALES.lin_vel_x,
            self.config.COMMANDS_SCALES.lin_vel_y,
            self.config.COMMANDS_SCALES.ang_vel_yaw
        ], dtype=np.float32)
        
        # ... (观测和动作变量初始化不变) ...
        self.qj = np.zeros(self.num_actions, dtype=np.float32)
        self.dqj = np.zeros(self.num_actions, dtype=np.float32)
        self.obs = np.zeros(60, dtype=np.float32) 
        self.cmd = np.array([0.0, 0.0, 0.0])
        self.low_state=LowState()
        # !!! 新增 !!!: 用于存储 EKF 估计的速度
        self.base_lin_vel = np.zeros(3, dtype=np.float32) 
        
        # ... (默认关节角度不变) ...
        self.default_dof_pos_policy_order = np.array([
            self.config.DEFAULT_JOINT_ANGLES['FL_hip_joint'], self.config.DEFAULT_JOINT_ANGLES['FL_thigh_joint'], self.config.DEFAULT_JOINT_ANGLES['FL_calf_joint'],
            self.config.DEFAULT_JOINT_ANGLES['FR_hip_joint'], self.config.DEFAULT_JOINT_ANGLES['FR_thigh_joint'], self.config.DEFAULT_JOINT_ANGLES['FR_calf_joint'],
            self.config.DEFAULT_JOINT_ANGLES['RL_hip_joint'], self.config.DEFAULT_JOINT_ANGLES['RL_thigh_joint'], self.config.DEFAULT_JOINT_ANGLES['RL_calf_joint'],
            self.config.DEFAULT_JOINT_ANGLES['RR_hip_joint'], self.config.DEFAULT_JOINT_ANGLES['RR_thigh_joint'], self.config.DEFAULT_JOINT_ANGLES['RR_calf_joint']
        ], dtype=np.float32)


        # ... (ROS 2 订阅) ...
        if simulation:
            self.low_state_sub=self.create_subscription(LowState,"/mujoco/lowstate",self.low_state_callback,10)
            print("reading data from simuation")
        else:    
            self.low_state_sub=self.create_subscription(LowState,"/lowstate",self.low_state_callback,10) 
            print("reading data from reality")

        # !!! 新增 !!!: 订阅 EKF 速度估计
        # self.velocity_sub = self.create_subscription(
        #     TwistStamped,
        #     "/ekf/velocity", # EKF 节点发布的话题
        #     self.velocity_callback,
        #     10)
        # !!! 修改 !!!: 订阅 MuJoCo 的真值速度
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            "/mujoco/ground_truth_velocity", # <--- 改成 MuJoCo 发布的
            self.velocity_callback,
            10)

        self.get_logger().info("Waiting for data")
        
        # 200Hz (0.005s) 定时器
        self.timer = self.create_timer(0.005, self.run) 
        
        self.target_torque_puber=self.create_publisher(Float32MultiArray,"/rl/target_torques",10)

        # !!! 新增: 速度控制状态变量 !!!
        self.target_speed_level = 0.2  # 初始速度设为 0.2 m/s
        self.last_y_state = 0          # 记录 Y 键上一帧状态
        self.last_b_state = 0          # 记录 B 键上一帧状态

    # !!! 新增 !!!: EKF 速度回调
    def velocity_callback(self, msg: TwistStamped):
        self.base_lin_vel[0] = msg.twist.linear.x
        self.base_lin_vel[1] = msg.twist.linear.y
        self.base_lin_vel[2] = msg.twist.linear.z

    def low_state_callback(self,msg:LowState):
        self.low_state=msg
    
    def _compute_sata_torques(self, raw_actions, dof_vel_policy_order):
        # ... (SATA 力矩计算逻辑不变, dt=0.005) ...
        actions_scaled = raw_actions * self.ACTION_SCALE
        torques_limits = self.torque_limits
        current_activation_sign = np.tanh(actions_scaled / torques_limits)
        self.activation_sign = (current_activation_sign - self.activation_sign) * self.GAMMA + self.activation_sign
        torques = self.activation_sign * torques_limits * (
            1 - np.sign(self.activation_sign) * dof_vel_policy_order / self.dof_vel_limits
        )
        dt = 0.005 
        self.motor_fatigue += np.abs(torques) * dt
        self.motor_fatigue *= self.BETA
        return torques.astype(np.float32)

    def run(self):
        
        if (self.cmd_sub.axes and self.cmd_sub.axes[2] == -1 and self.cmd_sub.axes[5] == -1):
            sys.exit()
            
        for i in range(12):
            self.qj[i] = self.low_state.motor_state[i].q
            self.dqj[i] = self.low_state.motor_state[i].dq
        
        # !!! 修复 1 !!!: 关节映射
        # (FR,FL,RR,RL) -> (FL,FR,RL,RR)
        policy_to_robot_map = np.array([3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8], dtype=np.int32)
        # 逆映射 (r2p) 恰好等于 (p2r)
        robot_to_policy_map = np.array([3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8], dtype=np.int32)

        qj_policy = self.qj[robot_to_policy_map]
        dqj_policy = self.dqj[robot_to_policy_map]
        
        quat = self.low_state.imu_state.quaternion
        ang_vel = np.array(self.low_state.imu_state.gyroscope, dtype=np.float32)
        
        self.cmd = np.zeros(3)
        self.left_button, self.right_button = self.cmd_sub.is_pressed()
        
        if self.left_button and self.right_button: # LB + RB 激活策略
            # 1. 获取原始摇杆数据 (方向)
            raw_lx, raw_ly = self.cmd_sub.get_left_stick()
            raw_az = self.cmd_sub.get_right_stick()
            
            # 2. 速度档位控制 (Y键加速, B键减速)
            # 确保 xbox_command.py 已经修改并暴露了 self.buttons
            if hasattr(self.cmd_sub, 'buttons') and len(self.cmd_sub.buttons) > 3:
                # --- 处理 Y 键 (Index 3): 加速 ---
                current_y_state = self.cmd_sub.buttons[3]
                # 检测上升沿 (从0变1的瞬间)
                if current_y_state == 1 and self.last_y_state == 0: 
                    self.target_speed_level += 0.1
                    print(f"Speed UP: {self.target_speed_level:.1f} m/s")
                self.last_y_state = current_y_state # 更新状态
                
                # --- 处理 B 键 (Index 1): 减速 ---
                current_b_state = self.cmd_sub.buttons[1]
                # 检测上升沿
                if current_b_state == 1 and self.last_b_state == 0: 
                    self.target_speed_level -= 0.1
                    if self.target_speed_level < 0.0: 
                        self.target_speed_level = 0.0 # 防止速度变为负数
                    print(f"Speed DOWN: {self.target_speed_level:.1f} m/s")
                self.last_b_state = current_b_state # 更新状态

            # 3. 计算最终指令 (归一化方向 * 设定速度)
            magnitude = np.sqrt(raw_lx**2 + raw_ly**2)
            target_vx = 0.0
            target_vy = 0.0
            
            # 设置死区，防止摇杆漂移导致机器人缓慢蠕动
            if magnitude > 0.1: 
                # 归一化方向向量 (Direction Vector)
                dir_x = raw_lx / magnitude
                dir_y = raw_ly / magnitude
                
                # 应用当前设定的速度档位
                target_vx = dir_x * self.target_speed_level
                target_vy = dir_y * self.target_speed_level
            
            # 4. 转向处理 (转向通常保留线性控制手感会更好)
            # 如果你也想固定转向速度，可以像上面一样处理，但建议保留线性
            target_wz = raw_az * 1.0 

            # 5. 赋值给 self.cmd
            self.cmd = np.array([target_vx, target_vy, target_wz])
        
        gravity_orientation = self.get_gravity_orientation(quat)
        
        # !!! 修复 2 !!!: 使用 EKF 的线速度
        obs_lin_vel = self.base_lin_vel * self.config.OBS_SCALES.lin_vel
        # obs_lin_vel = np.zeros(3, dtype=np.float32) # <--- 强制将线速度观测值设为 [0, 0, 0]
        obs_ang_vel = ang_vel * self.config.OBS_SCALES.ang_vel
        obs_dof_pos = (qj_policy - self.default_dof_pos_policy_order) * self.config.OBS_SCALES.dof_pos
        obs_dof_vel = dqj_policy * self.config.OBS_SCALES.dof_vel
        commands_scaled = self.cmd * self.CMD_SCALES

        self.obs = np.concatenate([
            obs_lin_vel,        # 3 (来自 EKF)
            obs_ang_vel,        # 3
            gravity_orientation,  # 3
            obs_dof_pos,        # 12
            obs_dof_vel,        # 12
            commands_scaled,    # 3
            self.last_torques,  # 12
            self.motor_fatigue  # 12
        ]).astype(np.float32)
        
        self.obs = np.clip(self.obs, -100, 100)
        
        raw_action = self.policy.get_action(self.obs) 
        final_torques_policy_order = self._compute_sata_torques(raw_action, dqj_policy)
        self.last_torques = final_torques_policy_order.copy()
        
        final_torques_robot_order = final_torques_policy_order[policy_to_robot_map]
        
        msg=Float32MultiArray()
        msg.data.extend(final_torques_robot_order.astype(np.float32).tolist())
        self.target_torque_puber.publish(msg)
        
    @staticmethod
    def get_gravity_orientation(quaternion):
        # MuJoCo 的四元数顺序通常是 [w, x, y, z]
        qw = quaternion[0]
        qx = quaternion[1]
        qy = quaternion[2]
        qz = quaternion[3]
        
        # 世界坐标系下的重力向量 [0, 0, -1]
        gravity_vec = np.array([0., 0., -1.], dtype=np.float32)
        
        # !!! 核心修改 !!! 
        # 使用共轭四元数 [w, -x, -y, -z] 来进行逆旋转 (World -> Body)
        # 也就是将 x, y, z 取反
        qv = np.array([-qx, -qy, -qz], dtype=np.float32) 
        
        # 下面的公式保持不变，但因为 qv 变了，计算结果就是逆旋转了
        uv = np.cross(qv, gravity_vec)
        uuv = np.cross(qv, uv)
        projected_gravity = gravity_vec + 2 * (qw * uv + uuv)
        
        return projected_gravity

    
def main():
    rclpy.init()
    
    try:
        from sata_config import Go2Config
        from sata_policy_loader import RLPolicy
    except ImportError:
        print("错误：无法导入 'sata_config' 或 'sata_policy_loader'。")
        print("请确保 sata_config.py 和 sata_policy_loader.py 在 src/deploy_rl_policy/scripts/ 目录下。")
        sys.exit(1)

    config = Go2Config()
    policy_path = "/home/qiwang/SATA/legged_gym/logs/SATA/exported/policies/policy_1.pt" 
    
    if not os.path.exists(policy_path):
        print(f"错误: 找不到策略文件: {policy_path}")
        sys.exit(1)
        
    policy = RLPolicy(policy_path)
    policy.policy.eval() 

    parser = argparse.ArgumentParser()
    parser.add_argument('--is_simulation', type=str, choices=["True", "False"], default="True")
    args = parser.parse_args()
    simulation = args.is_simulation == "True"
    
    reciever_node=dataReciever(config=config, policy=policy, simulation=simulation)
    rclpy.spin(reciever_node)
    reciever_node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()