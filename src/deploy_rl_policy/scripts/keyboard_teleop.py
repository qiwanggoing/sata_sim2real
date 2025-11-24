#!/usr/bin/env python3
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# --- 按键映射配置 ---
# 模拟 LB(4) + RB(5) 同时按下，用于激活策略
ENABLE_KEY = 'o' 

# 运动控制
MOVE_BINDINGS = {
    'w': (1, 0, 0),  # 前 (Lin X+)
    's': (-1, 0, 0), # 后 (Lin X-)
    'a': (0, 1, 0),  # 左 (Lin Y+)
    'd': (0, -1, 0), # 右 (Lin Y-)
    'q': (0, 0, 1),  # 左转 (Ang Z+)
    'e': (0, 0, -1), # 右转 (Ang Z-)
}

# 速度档位控制 (模拟 Y 和 B 键)
SPEED_BINDINGS = {
    'i': 3, # Y键 (Index 3) -> 加速
    'k': 1, # B键 (Index 1) -> 减速
}

STOP_KEY = 'x'
CTRL_C_KEY = '\x03'

class KeyboardJoystick(Node):
    def __init__(self):
        super().__init__('keyboard_joystick_node')
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)
        self.timer = self.create_timer(0.05, self.publish_joy) # 20Hz 发布
        
        # 状态变量
        self.enabled = False # 是否激活 (LB+RB)
        self.x_val = 0.0
        self.y_val = 0.0
        self.z_val = 0.0
        
        # 按键触发器 (用于模拟按一下松开)
        self.button_triggers = {1: 0, 3: 0} 

        print(self.get_instructions())
        self.settings = termios.tcgetattr(sys.stdin)

    def get_instructions(self):
        return """
        ---------- 键盘控制面板 (Sim2Sim) ----------
        
        【状态控制】
        o : 切换 激活/待机 (对应手柄 LB + RB)
        x : 立即停止并归零
        
        【移动控制】
        w/s : 前/后
        a/d : 左/右
        q/e : 转向
        
        【速度档位】
        i : 加速 (对应 Y 键)
        k : 减速 (对应 B 键)
        
        CTRL-C : 退出
        ------------------------------------------
        """

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_joy(self):
        # 1. 读取按键
        key = self.get_key()
        
        if key == CTRL_C_KEY:
            self.destroy_node()
            rclpy.shutdown()
            sys.exit()
            
        # 2. 处理按键逻辑
        if key == ENABLE_KEY:
            self.enabled = not self.enabled
            status = "激活 (Active)" if self.enabled else "待机 (Standby)"
            print(f"模式切换: {status}")
            
        elif key == STOP_KEY:
            self.x_val = 0.0
            self.y_val = 0.0
            self.z_val = 0.0
            print("停止指令!")

        elif key in MOVE_BINDINGS:
            # 简单的平滑处理：按下方向键增加数值，否则慢慢归零（可选，这里简化为直接赋值）
            vx, vy, vz = MOVE_BINDINGS[key]
            self.x_val = float(vx)
            self.y_val = float(vy)
            self.z_val = float(vz)
        
        elif key in SPEED_BINDINGS:
            idx = SPEED_BINDINGS[key]
            self.button_triggers[idx] = 1 # 标记按下
            print(f"触发功能键: {key}")
            
        else:
            # 如果没有按方向键，自动归零 (模拟摇杆回中)
            if key == '': 
                self.x_val = 0.0
                self.y_val = 0.0
                self.z_val = 0.0
        
        # 3. 构建 Joy 消息 (伪装成 Xbox 手柄)
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Axes (至少 6 个)
        # [0: LeftStick X, 1: LeftStick Y, 2: LT, 3: RightStick X, 4: RightStick Y, 5: RT]
        # xbox_command.py 读取: axes[1] (Y), axes[0] (X), axes[3] (Yaw)
        msg.axes = [0.0] * 6
        msg.axes[1] = self.x_val  # 前后
        msg.axes[0] = self.y_val  # 左右
        msg.axes[3] = self.z_val  # 转向
        
        # Buttons (至少 6 个)
        # [0: A, 1: B, 2: X, 3: Y, 4: LB, 5: RB]
        msg.buttons = [0] * 12
        
        # 激活状态 (LB + RB)
        if self.enabled:
            msg.buttons[4] = 1 # LB
            msg.buttons[5] = 1 # RB
            
        # 瞬时按钮 (速度调节)
        for idx, val in self.button_triggers.items():
            msg.buttons[idx] = val
        
        # 发布消息
        self.publisher_.publish(msg)
        
        # 重置瞬时按钮 (只触发一帧)
        for k in self.button_triggers:
            self.button_triggers[k] = 0

def main():
    rclpy.init()
    node = KeyboardJoystick()
    try:
        while rclpy.ok():
            node.publish_joy() # 主动在循环里调用
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()