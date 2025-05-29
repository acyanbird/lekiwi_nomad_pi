#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_publisher')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        self.get_logger().info('Cmd_vel publisher node has been started')

    def timer_callback(self):
        msg = Twist()
        # 设置线速度和角速度
        msg.linear.x = 0.5  # 前进速度 0.5 m/s
        msg.angular.z = 0.0  # 不旋转
        
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: linear.x=%f, angular.z=%f' % (msg.linear.x, msg.angular.z))

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()