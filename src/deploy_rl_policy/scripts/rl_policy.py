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
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            "/ekf/velocity", # EKF 节点发布的话题
            self.velocity_callback,
            10)

        self.get_logger().info("Waiting for data")
        
        # 200Hz (0.005s) 定时器
        self.timer = self.create_timer(0.005, self.run) 
        
        self.target_torque_puber=self.create_publisher(Float32MultiArray,"/rl/target_torques",10)

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
        self.left_button, self.right_button=self.cmd_sub.is_pressed()
        if self.left_button and self.right_button:
            linear_x, linear_y = self.cmd_sub.get_left_stick()
            angular_z = self.cmd_sub.get_right_stick()
            self.cmd = 0.7 * np.array([linear_x, linear_y, angular_z])
        
        gravity_orientation = self.get_gravity_orientation(quat)
        
        # !!! 修复 2 !!!: 使用 EKF 的线速度
        obs_lin_vel = self.base_lin_vel * self.config.OBS_SCALES.lin_vel
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
        # ... (SATA 重力向量计算不变) ...
        qw, qx, qy, qz = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        gravity_vec = np.array([0., 0., -1.], dtype=np.float32)
        q_calc = np.array([qx, qy, qz, qw], dtype=np.float32)
        w, x, y, z = q_calc[3], q_calc[0], q_calc[1], q_calc[2]
        qv = np.array([x, y, z], dtype=np.float32)
        uv = np.cross(qv, gravity_vec)
        uuv = np.cross(qv, uv)
        projected_gravity = gravity_vec + 2 * (w * uv + uuv)
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
    policy_path = "resources/policies/policy_1.pt" 
    
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