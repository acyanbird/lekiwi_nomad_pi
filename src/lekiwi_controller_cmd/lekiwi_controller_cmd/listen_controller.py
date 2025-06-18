#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from std_msgs.msg import Int32MultiArray

class CustomKiwiDriveController(Node): # Changed class name for clarity

    def __init__(self):
        super().__init__('custom_kiwi_drive_controller') # Changed node name
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',  # Standard topic for velocity commands
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # --- Robot Specific Parameters (modify according to your robot) ---
        self.wheel_radius = 0.073  # Radius of the wheels (meters)
        self.L_distance_to_center = 0.15  # Distance from robot center to wheel contact point (meters)

        # Pre-calculate trigonometric values for efficiency
        # Wheel ID 8 (back) - New Driving angle 270 degrees (drives right)
        self.phi8_rad = math.radians(270)
        self.cos_phi8 = math.cos(self.phi8_rad) # Should be 0 (approx)
        self.sin_phi8 = math.sin(self.phi8_rad) # Should be -1

        # Wheel ID 7 (left) - New Driving angle 150 degrees
        self.phi7_rad = math.radians(150)
        self.cos_phi7 = math.cos(self.phi7_rad) # sqrt(3)/2
        self.sin_phi7 = math.sin(self.phi7_rad) # 0.5

        # Wheel ID 9 (right) - New Driving angle 30 degrees
        self.phi9_rad = math.radians(30)
        self.cos_phi9 = math.cos(self.phi9_rad) # -sqrt(3)/2
        self.sin_phi9 = math.sin(self.phi9_rad) # 0.5

        self.servo_speed_pub = self.create_publisher(Int32MultiArray, '/servo_speed', 10)

        self.get_logger().info('Custom Kiwi Drive (3-Wheel Omni) Controller Node has been started.')
        self.get_logger().info(f'Parameters: Wheel Radius={self.wheel_radius}m, L={self.L_distance_to_center}m')
        self.get_logger().info(f'Driving angles (deg): Wheel8=270 (Right), Wheel7=150, Wheel9=30')
        

    def listener_callback(self, msg):
        vx = msg.linear.x      # Desired robot forward/backward velocity (m/s)
        vy = msg.linear.y      # Desired robot left/right strafe velocity (m/s) (positive is LEFT)
        wz = msg.angular.z     # Desired robot angular velocity (rad/s) (positive is CCW)

        self.get_logger().debug(f'Received cmd_vel: vx={vx:.3f}, vy={vy:.3f}, wz={wz:.3f}')

        # Calculate tangential speed (s_i) for each wheel
        # s_i = vx * cos(phi_i) + vy * sin(phi_i) + L * wz

        s8 = vx * self.cos_phi8 + vy * self.sin_phi8 + self.L_distance_to_center * wz
        s7 = vx * self.cos_phi7 + vy * self.sin_phi7 + self.L_distance_to_center * wz
        s9 = vx * self.cos_phi9 + vy * self.sin_phi9 + self.L_distance_to_center * wz

        # Calculate angular velocity (w_i in rad/s) for each wheel
        # w_i = s_i / wheel_radius
        try:
            w8_rad_s = s8 / self.wheel_radius
            w7_rad_s = s7 / self.wheel_radius
            w9_rad_s = s9 / self.wheel_radius
        except ZeroDivisionError:
            self.get_logger().error("Wheel radius is zero! Cannot calculate wheel speeds.")
            w8_rad_s = 0.0
            w7_rad_s = 0.0
            w9_rad_s = 0.0

        self.get_logger().info(f'Calculated wheel angular velocities (rad/s):')
        self.get_logger().info(f'  Wheel ID 8 (back, 270deg): {w8_rad_s:.3f}, and steps are: {self.rad_to_angle(w8_rad_s)}')
        self.get_logger().info(f'  Wheel ID 7 (left, 30deg): {w7_rad_s:.3f}, and steps are: {self.rad_to_angle(w7_rad_s)}')
        self.get_logger().info(f'  Wheel ID 9 (right, 150deg): {w9_rad_s:.3f}, and steps are: {self.rad_to_angle(w9_rad_s)}')

        step8 = self.rad_to_angle(w8_rad_s)
        step7 = self.rad_to_angle(w7_rad_s)
        step9 = self.rad_to_angle(w9_rad_s)

        # 发布到servo
        speed_msg = Int32MultiArray()
        speed_msg.data = [step7, step8, step9]  # 顺序：左7，后8，右9
        self.servo_speed_pub.publish(speed_msg)

        self.get_logger().info(f'发布舵机速度: {speed_msg.data}')
        
        

    def rad_to_angle(self, rad):
        return round(rad * 651.79)

def main(args=None):
    rclpy.init(args=args)
    custom_kiwi_controller = CustomKiwiDriveController()
    try:
        rclpy.spin(custom_kiwi_controller)
    except KeyboardInterrupt:
        custom_kiwi_controller.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        custom_kiwi_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()