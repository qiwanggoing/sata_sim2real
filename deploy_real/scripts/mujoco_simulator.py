#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import mujoco.viewer
import mujoco
import numpy as np
from unitree_go.msg import LowState,LowCmd
from pathlib import Path
from xbox_command import XboxController
from std_msgs.msg import Float32MultiArray
import threading
import time
from std_msgs.msg import Float32MultiArray


class MujocoSimulator(Node):
    def __init__(self):
        super().__init__("mujoco_simulator")
        self.cmd_sub = XboxController(self)  
        self.low_state_puber=self.create_publisher(LowState,"/mujoco/lowstate",10)
        self.pos_pub=self.create_publisher(Float32MultiArray,"/mujoco/pos",10)
        self.force_pub=self.create_publisher(Float32MultiArray,"/mujoco/force",10)
        self.torque_pub=self.create_publisher(Float32MultiArray,"/mujoco/torque",10)
        self.target_torque_suber=self.create_subscription(LowCmd,"/mujoco/lowcmd",self.target_torque_callback,10)

        self.step_counter = 0
        self.xml_path="/home/song/mujoco-3.3.0-linux-x86_64/mujoco-3.3.0/model/go2/go2.xml"
        # Initialize Mujoco
        self.init_mujoco()
        self.target_dof_pos=None
        self.tau=[0]*12
        # Load policy
        self.timer = self.create_timer(0.002, self.publish_sensor_data)
        self.running=True
        self.kps=np.array([20.0]*12)
        self.kds=np.array([1.0]*12)
        self.recieve_data=False
        self.sim_thread=threading.Thread(target=self.step_simulation)
        self.sim_thread.start()

    def init_mujoco(self):
        """Initialize Mujoco model and data"""
        
        self.m = mujoco.MjModel.from_xml_path(self.xml_path)
        self.d = mujoco.MjData(self.m)
        # self.d.qpos[3:7]=[
        #       0.9932722449302673,
        #     0.008041736669838428,
        #     0.0063380408100783825,
        #     -0.11535090208053589]
   
        # laydown=[-0.6452068 ,0.84766394 ,-2.7774005 ,  0.674909   , 0.92326707 ,-2.8047292,
        #         -0.7110552,   0.43561167, -2.7976365 ,  0.7677953,   0.84905136, -2.8131175]
                
        # for i in range(12):
        #     self.d.qpos[7+i] =laydown[i] 
        self.m.opt.timestep = 0.005 
        self.viewer = mujoco.viewer.launch_passive(self.m, self.d)
        self.control_decimal=0
        print("Number of qpos:", self.m.nq)
        print("Joint order:")
        for i in range(self.m.njnt):
            print(f"{i}: {self.m.joint(i).name}")
   
    def target_pos_callback(self,msg):
        self.recieve_data=True
        self.target_dof_pos=msg.data[:12]
        
    def target_torque_callback(self,msg):
        self.recieve_data=True
        for i in range(12):
            self.tau[i]=msg.motor_cmd[i].tau
    def step_simulation(self):
        while self.viewer.is_running() and self.running :
            if not self.recieve_data:
                continue
            step_start=time.time()
            self.control_decimal+=1
            """Main simulation step (executed in another thread)"""
            # tau = self.pd_control(
            # self.target_dof_pos,
            # self.d.qpos[7:19],
            # self.kps,
            # self.d.qvel[6:18],
            # self.kds
            # ).astype(np.float32)
            # tau=tau[sequence]
            # self.d.ctrl[:] = tau
            self.d.ctrl[:]=self.tau
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
        low_state_msg=LowState()
        # print(type(self.d.qpos))
        # print(type(self.d.sensordata[55:67]))
        for i in range(12):
            low_state_msg.motor_state[i].q=self.d.qpos[7+i]
            low_state_msg.motor_state[i].dq=self.d.qvel[6+i]
        low_state_msg.imu_state.quaternion=self.d.qpos[3:7].astype(np.float32)
        low_state_msg.imu_state.gyroscope=self.d.sensordata[40:43].astype(np.float32)
        self.low_state_puber.publish(low_state_msg)
        pos=Float32MultiArray()
        sequence_joint=[0,1,2,4,5,6,3,10,11,12,7,8,9,16,17,18,13,14,15]
        # print(len(self.d.qpos[sequence_joint].astype(np.float32)))
        # pos.data=self.d.qpos[sequence_joint].astype(np.float32)
        pos.data=self.d.qpos[:19].tolist()
        self.pos_pub.publish(pos)
        Torque=Float32MultiArray()
        # sequence_torque=[3,4,5,0,1,2,9,10,11,7,8,9]
        # Torque.data=self.d.sensordata[24:36][sequence_torque]
        self.torque_pub.publish(Torque)
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
        torques=(target_q - q) * kp -  dq * kd
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

