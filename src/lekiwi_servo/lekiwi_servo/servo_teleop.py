#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import termios
import tty

class ServoTeleop(Node):
    def __init__(self):
        super().__init__('servo_teleop')
        
        # 创建发布者
        self.key_publisher = self.create_publisher(String, 'servo_key', 10)
        
        # 保存终端设置
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('舵机遥操作节点已启动')
        self.get_logger().info('使用 WASD 控制舵机，空格键停止，Ctrl+C 退出')
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if key in ['w', 'a', 's', 'd', ' ']:
                    msg = String()
                    msg.data = key
                    self.key_publisher.publish(msg)
                    self.get_logger().info(f'发送按键: {key}')
                elif key == '\x03':  # Ctrl+C
                    break
        finally:
            # 停止舵机
            msg = String()
            msg.data = ' '
            self.key_publisher.publish(msg)
            self.get_logger().info('停止舵机')

def main(args=None):
    rclpy.init(args=args)
    teleop = ServoTeleop()
    
    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 