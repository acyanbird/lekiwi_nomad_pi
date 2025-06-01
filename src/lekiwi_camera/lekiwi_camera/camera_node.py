#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        # 声明参数
        self.declare_parameter('device_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        
        # 获取参数
        self.device_id = self.get_parameter('device_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        
        # 创建发布者
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        
        # 检查设备是否存在
        device_path = f'/dev/video{self.device_id}'
        if not os.path.exists(device_path):
            self.get_logger().error(f'Camera device {device_path} does not exist!')
            return
            
        # 初始化OpenCV
        self.cap = cv2.VideoCapture(self.device_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera device {self.device_id}')
            return
            
        # 设置摄像头参数
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        # 验证设置是否成功
        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        self.get_logger().info(f'Camera initialized with:')
        self.get_logger().info(f'  Device: {device_path}')
        self.get_logger().info(f'  Resolution: {actual_width}x{actual_height}')
        self.get_logger().info(f'  FPS: {actual_fps}')
        
        # 创建CV桥接器
        self.bridge = CvBridge()
        
        # 创建定时器
        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info('Camera node has been started')

    def timer_callback(self):
        if not self.cap.isOpened():
            self.get_logger().error('Camera is not opened!')
            return
            
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return
            
        if frame is None or frame.size == 0:
            self.get_logger().warn('Received empty frame')
            return
            
        try:
            # 将OpenCV图像转换为ROS消息
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error converting image: {str(e)}')

    def __del__(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error in main: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
