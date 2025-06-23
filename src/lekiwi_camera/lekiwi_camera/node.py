#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
def main(args=None):
    # --- 1. ROS 初始化 ---
    rclpy.init(args=args)
    node = Node('camera_node_manual_set')
    
    node.declare_parameter('device_id', 0)
    DEVICE_ID = node.get_parameter('device_id').get_parameter_value().integer_value

    node.get_logger().info(f"--- Manual Setting Camera Node ---")
    node.get_logger().info(f"Attempting to open camera with ID: {DEVICE_ID}")

    # --- 2. 开启摄像头 ---
    cap = cv2.VideoCapture(DEVICE_ID, cv2.CAP_V4L2)

    if not cap.isOpened():
        node.get_logger().error(f"Fatal: Cannot open camera with ID {DEVICE_ID}.")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info(f"[SUCCESS] Camera {DEVICE_ID} opened. Now attempting to set parameters...")

    # --- 新增：设置您指定的参数 ---
    success = cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y', 'U', 'Y', 'V'))
    if not success:
        node.get_logger().warn("Failed to set FOURCC to YUYV.")

    # success = cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # if not success:
    #     node.get_logger().warn("Failed to set frame width to 640.")

    # success = cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # if not success:
    #     node.get_logger().warn("Failed to set frame height to 480.")

    # success = cap.set(cv2.CAP_PROP_FPS, 10)
    # if not success:
    #     node.get_logger().warn("Failed to set FPS to 10.")
    # ------------------------------------

    # 获取并打印设置后的实际参数，用于验证
    actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc_str = "".join([chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4)])
    actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    actual_fps = cap.get(cv2.CAP_PROP_FPS)

    node.get_logger().info(f"--- Verifying Settings ---")
    node.get_logger().info(f"  Actual FOURCC: {fourcc_str}")
    node.get_logger().info(f"  Actual Resolution: {int(actual_width)}x{int(actual_height)}")
    node.get_logger().info(f"  Actual FPS: {actual_fps}")
    node.get_logger().info(f"--------------------------")

    # --- 3. 创建 ROS 组件 ---
    publisher = node.create_publisher(Image, 'camera/image_raw', 10)
    bridge = CvBridge()

    node.get_logger().info("Starting publishing loop...")
    
    # --- 4. 主循环 ---
    try:
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                node.get_logger().warn("Failed to read frame from camera stream, retrying...")
                rclpy.spin_once(node, timeout_sec=0.01) 
                continue

            ros_image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            ros_image_msg.header.stamp = node.get_clock().now().to_msg()
            ros_image_msg.header.frame_id = "camera"
            publisher.publish(ros_image_msg)

    except KeyboardInterrupt:
        node.get_logger().info("User stopped the node with Ctrl+C.")
    finally:
        # --- 5. 清理 ---
        node.get_logger().info("Releasing camera resource...")
        cap.release()
        node.get_logger().info("Destroying node...")
        node.destroy_node()
        rclpy.shutdown()
        node.get_logger().info("Script finished.")


if __name__ == '__main__':
    main()