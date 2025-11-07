#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import mujoco.viewer
import mujoco
import numpy as np
from unitree_go.msg import LowState,LowCmd
from pathlib import Path
from xbox_command import XboxController # 假设 xbox_command 在同一目录
from std_msgs.msg import Float32MultiArray
import threading
import time
from std_msgs.msg import Float32MultiArray
project_root=Path(__file__).parents[4]

class MujocoSimulator(Node):
    def __init__(self):
        super().__init__("mujoco_simulator")
        self.cmd_sub = XboxController(self)  
        self.low_state_puber=self.create_publisher(LowState,"/mujoco/lowstate",10)
        self.pos_pub=self.create_publisher(Float32MultiArray,"/mujoco/pos",10)
        self.force_pub=self.create_publisher(Float32MultiArray,"/mujoco/force",10)
        self.torque_pub=self.create_publisher(Float32MultiArray,"/mujoco/torque",10)
        
        # !!!修改!!!: 订阅 /mujoco/lowcmd (回调函数将被修改)
        self.target_torque_suber=self.create_subscription(LowCmd,"/mujoco/lowcmd",self.target_torque_callback,10)

        self.step_counter = 0
        self.xml_path=project_root/"resources"/"go2"/"scene_terrain.xml"
        
        # 初始化 Mujoco
        self.init_mujoco()
        self.tau=[0.0]*12 # tau 将由 target_torque_callback 直接计算和设置
        
        self.timer = self.create_timer(0.002, self.publish_sensor_data)
        
        # !!!删除!!!: self.timer2 和 self.update_tau
        # (PD控制和力矩应用将在 target_torque_callback 中处理)
        
        self.running=True
        self.recieve_data=False
        self.sim_thread=threading.Thread(target=self.step_simulation)
        self.sim_thread.start()

    def init_mujoco(self):
        """Initialize Mujoco model and data"""
        
        self.m = mujoco.MjModel.from_xml_path(str(self.xml_path))
        self.d = mujoco.MjData(self.m)
        self.m.opt.timestep = 0.005 # 保持 0.005s (200Hz)
        self.viewer = mujoco.viewer.launch_passive(self.m, self.d)
        print("Number of qpos:", self.m.nq)
        print("Joint order:")
        for i in range(self.m.njnt):
            print(f"{i}: {self.m.joint(i).name}")
   
    # !!!修改!!!: target_torque_callback 现在处理 PD 和 纯力矩
    def target_torque_callback(self,msg: LowCmd):
        self.recieve_data=True
        temp_tau = [0.0] * 12 # 临时力矩列表
        
        for i in range(12):
            kp = msg.motor_cmd[i].kp
            kd = msg.motor_cmd[i].kd
            
            if kp > 0:
                # 1. PD 命令 (用于站立/趴下)
                target_q = msg.motor_cmd[i].q
                target_dq = msg.motor_cmd[i].dq 
                
                # 从MuJoCo数据中获取当前状态
                current_q = self.d.qpos[7+i]
                current_dq = self.d.qvel[6+i]
                
                # 计算PD力矩 (静态 PD 控制)
                pd_torque = self.pd_control(target_q, current_q, kp, current_dq, kd)
                
                # 加上前馈力矩 (如果有的话)
                temp_tau[i] = pd_torque + msg.motor_cmd[i].tau
                
            else:
                # 2. 纯力矩命令 (Kp=0, Kd=0) (用于RL策略)
                temp_tau[i] = msg.motor_cmd[i].tau
        
        # 将计算好的力矩赋给 self.tau (供仿真线程使用)
        self.tau = temp_tau
        
    # !!!删除!!!: update_tau 方法
    
    def step_simulation(self):
        while self.viewer.is_running() and self.running :
            if not self.recieve_data:
                continue
            step_start=time.time()
            
            # !!!修改!!!: 直接应用 self.tau (由回调函数计算)
            self.d.ctrl[:]=self.tau 
            
            Torque=Float32MultiArray()
            Torque.data=self.tau
            self.torque_pub.publish(Torque)
            
            # Mujoco step
            mujoco.mj_step(self.m, self.d)  
            
            # Sync Mujoco viewer
            self.viewer.sync()
            
            time_until_next_step = self.m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
            
    def stop_simulation(self):
        self.running=False
        self.sim_thread.join()


    def publish_sensor_data(self):
        # ... (发布传感器数据的逻辑保持不变) ...
        low_state_msg=LowState()
        for i in range(12):
            low_state_msg.motor_state[i].q=self.d.qpos[7+i]
            low_state_msg.motor_state[i].dq=self.d.qvel[6+i]
        low_state_msg.imu_state.quaternion=self.d.qpos[3:7].astype(np.float32)
        low_state_msg.imu_state.gyroscope=self.d.sensordata[40:43].astype(np.float32)
        self.low_state_puber.publish(low_state_msg)
        pos=Float32MultiArray()
        pos.data=self.d.qpos[:19].tolist()
        self.pos_pub.publish(pos)
  
        Force=Float32MultiArray()
        f1=self.d.sensordata[55:55+3]+[0,0,0]
        f2=self.d.sensordata[55+3:55+6]+[0,0,0]
        f3=self.d.sensordata[55+6:55+9]+[0,0,0]
        f4=self.d.sensordata[55+9:55+12]+[0,0,0]
        # Force.data=f1+f2+f3+f4
        # self.force_pub.publish(Force)
    
    @staticmethod
    def pd_control(target_q, q, kp, dq, kd):
        """Calculates torques from position commands"""
        # 注意：原始代码的pd是 (target_q - q) * kp - dq * kd
        # 我们这里假设 target_dq = 0
        torques = (target_q - q) * kp - dq * kd
        return torques
    
def main(args=None):
    rclpy.init(args=args)
    node = MujocoSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_simulation()
        node.viewer.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()