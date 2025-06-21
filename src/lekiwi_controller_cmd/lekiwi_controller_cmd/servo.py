#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
import tf2_ros
import math
import numpy as np
from scservo_sdk import *
from rclpy.time import Duration
from std_msgs.msg import String, Int32MultiArray
import random

# Default setting
BAUDRATE                = 1000000           # SCServo default baudrate : 1000000
DEVICENAME              = '/dev/ttyACM0'    # Check which port is being used on your controller

protocol_end            = 0                 # SCServo bit end(STS/SMS=0, SCS=1)
COMM_SUCCESS = 0  # tx or rx packet communication success
ADDR_SCS_CURRENT_LOCATION = 56

# 速度映射 (ID7, ID8, ID9)
SPEED_MAP = {
    'w': [-1016, 0, 1016],     # 前进
    's': [1016, 0, -1016],     # 后退
    'a': [890, -1780, 890],    # 左转
    'd': [-890, 1780, -890],   # 右转
    ' ': [0, 0, 0],            # 停止
    'q': [450, 450, 450],    # spin left
    'e': [-450, -450, -450] #spin right
}

# address
ADDR_SCS_GOAL_SPEED        = 46
ADDR_SCS_OPERATION_MODE    = 33

class OmniWheelOdomNode(Node):
    def __init__(self):
        super().__init__('omni_wheel_odom_node')
        
        # 声明参数 - 全向三轮机器人参数
        self.declare_parameter('wheel_radius', 0.073)  # 轮子半径 (米)
        self.declare_parameter('robot_radius', 0.15)  # 机器人半径 (轮子到中心的距离)
        self.declare_parameter('wheel_angle_7', 150 * math.pi / 180)  # 第一个轮子的角度 (弧度)
        self.declare_parameter('wheel_angle_8', 270 * math.pi / 180)  # 第二个轮子的角度 (弧度)
        self.declare_parameter('wheel_angle_9', 30 * math.pi / 180)  # 第三个轮子的角度 (弧度)
        self.declare_parameter('ticks_per_revolution', 4095)  # 每转编码器脉冲数
        self.declare_parameter('publish_rate', 10.0)  # 发布频率 (Hz)
        
        # 获取参数
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        self.wheel_angle_1 = self.get_parameter('wheel_angle_7').get_parameter_value().double_value
        self.wheel_angle_2 = self.get_parameter('wheel_angle_8').get_parameter_value().double_value
        self.wheel_angle_3 = self.get_parameter('wheel_angle_9').get_parameter_value().double_value
        self.ticks_per_revolution = self.get_parameter('ticks_per_revolution').get_parameter_value().integer_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # 初始化变量
        self.encoder_prev = [0, 0, 0]  # 三个轮子的上一次编码器值
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.last_debug_time = self.get_clock().now()  # 上次调试输出时间
        
        # 创建发布者
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # 创建 TF2 广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # 创建定时器
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_odom)
        
        # 模拟编码器数据 (实际应用中应该从硬件读取)
        
        # 计算运动学矩阵 (全向轮运动学)
        self.setup_kinematics_matrix()
        
        self.get_logger().info("全向三轮 Odom Node 已启动")
        self.get_logger().info(f"轮子半径: {self.wheel_radius} m")
        self.get_logger().info(f"机器人半径: {self.robot_radius} m")
        self.get_logger().info(f"轮子角度: {math.degrees(self.wheel_angle_1):.1f}°, {math.degrees(self.wheel_angle_2):.1f}°, {math.degrees(self.wheel_angle_3):.1f}°")
        self.get_logger().info(f"编码器分辨率: {self.ticks_per_revolution} ticks/rev")
        self.get_logger().info(f"发布频率: {self.publish_rate} Hz")

        
        self.packetHandler = PacketHandler(protocol_end)
        self.portHandler = PortHandler(DEVICENAME)
        
        # 尝试打开串口
        if self.portHandler.openPort():
            self.get_logger().info(f"成功打开串口: {DEVICENAME}")
        else:
            self.get_logger().warn(f"无法打开串口: {DEVICENAME}，将使用模拟数据")
            
        if self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info(f"成功设置波特率: {BAUDRATE}")
        else:
            self.get_logger().warn(f"无法设置波特率: {BAUDRATE}，将使用模拟数据")

        self.SCS_IDS = [7, 8, 9]

        # 初始化编码器数据
        if self.portHandler.is_open:
            self.get_logger().info("串口连接已建立，尝试读取舵机数据...")
            for i in self.SCS_IDS:
                scs_present_position, scs_comm_result, scs_error = self.packetHandler.read2ByteTxRx(
                    self.portHandler, i, ADDR_SCS_CURRENT_LOCATION)
                if scs_comm_result != COMM_SUCCESS:
                    self.get_logger().warn(f"舵机 {i} 通信失败: {self.packetHandler.getTxRxResult(scs_comm_result)}")
                elif scs_error != 0:
                    self.get_logger().warn(f"舵机 {i} 错误: {self.packetHandler.getRxPacketError(scs_error)}")

                self.encoder_prev[i] = scs_present_position
                # 设置运行模式为速度模式
                scs_comm_result = self.packetHandler.write1ByteTxOnly(
                self.portHandler, i, ADDR_SCS_OPERATION_MODE, 1)

                # 等待一下确保模式设置完成
                self.get_clock().sleep_for(Duration(seconds=0.1))
                    
                # 验证运行模式
                current_mode, scs_comm_result, scs_error = self.packetHandler.read1ByteTxRx(
                    self.portHandler, i, ADDR_SCS_OPERATION_MODE)
                    
                if scs_comm_result != COMM_SUCCESS:
                    self.get_logger().error(f'Failed to read operation mode for servo {servo_id}: {self.packetHandler.getTxRxResult(scs_comm_result)}')
                    continue
                elif scs_error != 0:
                    self.get_logger().error(f'Error reading operation mode for servo {servo_id}: {self.packetHandler.getRxPacketError(scs_error)}')
                    continue
                    
                self.get_logger().info(f'舵机 {i} 运行模式: {current_mode}')

                # 设置初始速度为0
                scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(
                    self.portHandler, i, ADDR_SCS_GOAL_SPEED, 0)
                    
                if scs_comm_result != COMM_SUCCESS:
                    self.get_logger().error(f'Failed to set initial speed for servo {servo_id}: {self.packetHandler.getTxRxResult(scs_comm_result)}')
                    continue
                elif scs_error != 0:
                    self.get_logger().error(f'Error setting initial speed for servo {servo_id}: {self.packetHandler.getRxPacketError(scs_error)}')
                    continue
                    
                self.get_logger().info(f'舵机 {i} 初始化完成')
            
        else:
            self.get_logger().info("使用模拟编码器数据")
            self.encoder_prev = [2048, 2048, 2048]  # 模拟初始位置
        
        # 创建按键订阅者
        self.key_subscription = self.create_subscription(
            String,
            'servo_key',
            self.key_callback,
            10)
            
        # 在 __init__ 里，在 self.key_subscription 后添加：
        self.speed_subscription = self.create_subscription(
            Int32MultiArray,
            '/servo_speed',
            self.speed_callback,
            10)
            
        self.running = True  # 添加运行状态标志

        self.get_logger().info("OmniWheelOdomNode 初始化完成，开始发布里程计消息...")

    
    def setup_kinematics_matrix(self):
        """
        设置全向轮运动学矩阵
        用于将轮子速度转换为机器人速度
        """
        # 全向轮运动学矩阵
        # [v_x]   [cos(θ1)  sin(θ1)  L] [ω1]
        # [v_y] = [cos(θ2)  sin(θ2)  L] [ω2]
        # [ω_z]   [cos(θ3)  sin(θ3)  L] [ω3]
        
        self.kinematics_matrix = np.array([
            [math.cos(self.wheel_angle_1), math.sin(self.wheel_angle_1), self.robot_radius],
            [math.cos(self.wheel_angle_2), math.sin(self.wheel_angle_2), self.robot_radius],
            [math.cos(self.wheel_angle_3), math.sin(self.wheel_angle_3), self.robot_radius]
        ])
        
        # 计算逆矩阵用于从轮子速度计算机器人速度
        self.inverse_kinematics_matrix = np.linalg.inv(self.kinematics_matrix)
        
        self.get_logger().info("运动学矩阵已设置")
    
    def read_wheel_encoders(self):
        """
        读取三个轮子的编码器数据
        舵机位置在0到4095之间变化
        """
        encoders = [0, 0, 0]
        
        if not self.portHandler.is_open:
            # 使用模拟数据
            for i in range(3):
                # 模拟编码器值在2048附近小幅变化
                encoders[i] = max(0, min(4095, self.encoder_prev[i] + random.randint(-10, 10)))
            return encoders
        
        for i, scs_id in enumerate(self.SCS_IDS):
            scs_present_position, scs_comm_result, scs_error = self.packetHandler.read2ByteTxRx(
                self.portHandler, scs_id, ADDR_SCS_CURRENT_LOCATION)
            
            if scs_comm_result != COMM_SUCCESS:
                self.get_logger().warn(f"舵机 {scs_id} 通信失败: {self.packetHandler.getTxRxResult(scs_comm_result)}")
                # 如果通信失败，使用上一次的值
                encoders[i] = self.encoder_prev[i] if i < len(self.encoder_prev) else 0
            elif scs_error != 0:
                self.get_logger().warn(f"舵机 {scs_id} 错误: {self.packetHandler.getRxPacketError(scs_error)}")
                # 如果读取错误，使用上一次的值
                encoders[i] = self.encoder_prev[i] if i < len(self.encoder_prev) else 0
            else:
                # 确保位置值在0-4095范围内
                encoders[i] = max(0, min(4095, scs_present_position))
        
        self.encoders = encoders
        return self.encoders.copy()
    
    def calculate_odometry(self, encoders):
        """
        根据三个轮子的编码器数据计算里程计
        使用全向轮运动学
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt <= 0:
            return None
        
        # 计算编码器增量，处理跨越零点的情况
        encoder_deltas = []
        for i in range(3):
            current = encoders[i]
            previous = self.encoder_prev[i]
            
            # 计算原始增量
            delta = current - previous
            
            # 处理跨越零点的情况
            # 如果增量超过编码器范围的一半，说明可能跨越了零点
            if delta > self.ticks_per_revolution / 2:
                # 从4095跳转到0附近，实际应该是负向运动
                delta = delta - self.ticks_per_revolution
                self.get_logger().debug(f"舵机{self.SCS_IDS[i]} 跨越零点(正向): {previous} -> {current}, 修正增量: {delta}")
            elif delta < -self.ticks_per_revolution / 2:
                # 从0跳转到4095附近，实际应该是正向运动
                delta = delta + self.ticks_per_revolution
                self.get_logger().debug(f"舵机{self.SCS_IDS[i]} 跨越零点(负向): {previous} -> {current}, 修正增量: {delta}")
            
            encoder_deltas.append(delta)
        
        # 转换为轮子角速度 (rad/s)
        wheel_angular_velocities = []
        for delta in encoder_deltas:
            # 将编码器脉冲转换为角速度
            angular_velocity = (delta / self.ticks_per_revolution) * 2 * math.pi / dt
            wheel_angular_velocities.append(angular_velocity)
        
        # 使用运动学矩阵计算机器人速度
        wheel_velocities = np.array(wheel_angular_velocities) * self.wheel_radius
        robot_velocities = np.dot(self.inverse_kinematics_matrix, wheel_velocities)
        
        v_x = robot_velocities[0]  # 机器人 X 方向速度
        v_y = robot_velocities[1]  # 机器人 Y 方向速度
        omega_z = robot_velocities[2]  # 机器人角速度
        
        # 更新位置 (使用欧拉积分)
        # 在机器人坐标系中的位移
        dx_robot = v_x * dt
        dy_robot = v_y * dt
        dtheta = omega_z * dt
        
        # 转换到世界坐标系
        dx_world = dx_robot * math.cos(self.theta) - dy_robot * math.sin(self.theta)
        dy_world = dx_robot * math.sin(self.theta) + dy_robot * math.cos(self.theta)
        
        # 更新位置
        self.x += dx_world
        self.y += dy_world
        self.theta += dtheta
        
        # 标准化角度到 [-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # 更新上一次的编码器值
        self.encoder_prev = encoders.copy()
        self.last_time = current_time
        
        return v_x, v_y, omega_z
    
    def publish_odom(self):
        """
        发布里程计信息
        """
        # 读取编码器数据
        encoders = self.read_wheel_encoders()
        
        # 计算里程计
        velocities = self.calculate_odometry(encoders)
        if velocities is None:
            self.get_logger().warn("里程计计算失败，跳过本次发布")
            return
        
        v_x, v_y, omega_z = velocities
        
        # 创建里程计消息
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        # 设置位置
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # 设置方向 (四元数)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # 设置速度
        odom_msg.twist.twist.linear.x = v_x
        odom_msg.twist.twist.linear.y = v_y
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = omega_z
        
        # 发布里程计消息
        self.odom_pub.publish(odom_msg)
        
        # 发布 TF 变换
        self.publish_tf()
        
        # 发布关节状态
        self.publish_joint_states(encoders)
        
        # 打印调试信息，每秒一次
        current_time = self.get_clock().now()
        debug_dt = (current_time - self.last_debug_time).nanoseconds / 1e9
        
        if debug_dt >= 1.0:  # 每秒输出一次调试信息
            self.get_logger().info(f"=== 里程计调试信息 ===")
            self.get_logger().info(f"位置: x={self.x:.3f} m, y={self.y:.3f} m, theta={math.degrees(self.theta):.1f}°")
            self.get_logger().info(f"速度: v_x={v_x:.3f} m/s, v_y={v_y:.3f} m/s, omega={math.degrees(omega_z):.1f}°/s")
            self.get_logger().info(f"编码器值: {encoders}")
            self.get_logger().info(f"编码器增量: {[encoders[i] - self.encoder_prev[i] for i in range(3)]}")
            self.get_logger().info(f"已发布 odom 消息")
            self.get_logger().info(f"========================")
            self.last_debug_time = current_time
    
    def publish_tf(self):
        """
        发布 TF 变换
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)
    
    def publish_joint_states(self, encoders):
        """
        发布关节状态 (三个轮子的位置)
        """
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ["wheel_7_joint", "wheel_8_joint", "wheel_9_joint"]
        
        # 将编码器脉冲转换为关节角度 (弧度)
        joint_angles = []
        for encoder in encoders:
            angle = (encoder / self.ticks_per_revolution) * 2 * math.pi
            joint_angles.append(angle)
        
        joint_state.position = joint_angles
        joint_state.velocity = [0.0, 0.0, 0.0]  # 可以计算实际速度
        joint_state.effort = [0.0, 0.0, 0.0]    # 可以读取实际扭矩
        
        self.joint_state_pub.publish(joint_state)

    def key_callback(self, msg):
        key = msg.data
        if key == 'f':  # 处理 f 键
            # 先停止所有舵机
            for servo_id in self.servo_ids:
                self.set_speed(servo_id, 0)
            self.get_logger().info('收到退出命令，停止所有舵机')
            self.running = False
            # 关闭串口
            if hasattr(self, 'portHandler'):
                self.portHandler.closePort()
            # 退出节点
            rclpy.shutdown()
        elif key in SPEED_MAP:
            speeds = SPEED_MAP[key]
            for servo_id, speed in zip(self.servo_ids, speeds):
                self.set_speed(servo_id, speed)
            self.get_logger().info(f'按键 {key} 对应速度: {speeds}')
    
    def speed_callback(self, msg):
        speeds = msg.data  # [id7速度, id8速度, id9速度]
        for servo_id, speed in zip(self.servo_ids, speeds):
            self.set_speed(servo_id, speed)
        self.get_logger().info(f'收到 /servo_speed 速度: {speeds}')
    
    def set_speed(self, servo_id, speed):
        # 限制最大速度
        speed = min(speed, 3250)
        speed = max(speed, -3250)
        
        # 处理速度值
        if speed < 0:
            speed = abs(speed) | 0x8000  # 设置方向位
            self.get_logger().info(f'舵机 {servo_id} 负速度处理: 原始值={speed & 0x7FFF}, 处理后={speed}')
        else:
            self.get_logger().info(f'舵机 {servo_id} 正速度处理: 值={speed}')
        
        # 发送速度命令
        scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(
            self.portHandler, servo_id, ADDR_SCS_GOAL_SPEED, speed)
            
        if scs_comm_result != COMM_SUCCESS:
            self.get_logger().error(f'设置舵机 {servo_id} 速度失败: {self.packetHandler.getTxRxResult(scs_comm_result)}')
            return False
        elif scs_error != 0:
            self.get_logger().error(f'舵机 {servo_id} 错误: {self.packetHandler.getRxPacketError(scs_error)}')
            return False
            
        # 等待一下确保速度设置完成
        self.get_clock().sleep_for(Duration(seconds=0.1))
            
        # 验证速度是否设置成功

def main(args=None):
    rclpy.init(args=args)
    node = OmniWheelOdomNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断，正在关闭节点...")
    finally:
        # 关闭串口连接
        if hasattr(node, 'portHandler') and node.portHandler.is_open:
            node.portHandler.closePort()
            node.get_logger().info("串口连接已关闭")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 