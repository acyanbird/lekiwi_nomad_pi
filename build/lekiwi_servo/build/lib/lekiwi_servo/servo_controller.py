#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from std_msgs.msg import String
from scservo_sdk import *

# Default setting
BAUDRATE                = 1000000           # SCServo default baudrate : 1000000
DEVICENAME              = '/dev/ttyACM0'    # Check which port is being used on your controller

protocol_end            = 0                 # SCServo bit end(STS/SMS=0, SCS=1)
COMM_SUCCESS = 0  # tx or rx packet communication success

# 速度映射 (ID7, ID8, ID9)
SPEED_MAP = {
    'w': [-1016, 0, 1016],     # 前进
    's': [1016, 0, -1016],     # 后退
    'a': [890, -1780, 890],    # 左转
    'd': [-890, 1780, -890],   # 右转
    ' ': [0, 0, 0],            # 停止
}

# address
ADDR_SCS_GOAL_SPEED        = 46
ADDR_SCS_OPERATION_MODE    = 33

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        
        # 创建参数
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1000000)
        
        # 获取参数
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        
        # 舵机ID列表
        self.servo_ids = [7, 8, 9]
        
        # 初始化SDK
        self.protocol_end = 0  # SCServo bit end(STS/SMS=0, SCS=1)
        self.portHandler = PortHandler(self.port)
        self.packetHandler = PacketHandler(self.protocol_end)
        
        # 初始化舵机
        if not self.init_servos():
            self.get_logger().error('舵机初始化失败')
            return
            
        # 创建按键订阅者
        self.key_subscription = self.create_subscription(
            String,
            'servo_key',
            self.key_callback,
            10)
            
        self.get_logger().info('舵机控制器节点已启动')
    
    def init_servos(self):
        # 打开串口
        if not self.portHandler.openPort():
            self.get_logger().error(f'Failed to open port {self.port}')
            return False
        self.get_logger().info(f'Succeeded to open port {self.port}')
            
        # 设置波特率
        if not self.portHandler.setBaudRate(self.baudrate):
            self.get_logger().error(f'Failed to set baudrate {self.baudrate}')
            return False
        self.get_logger().info(f'Succeeded to set baudrate {self.baudrate}')
            
        # 初始化每个舵机
        for servo_id in self.servo_ids:
            self.get_logger().info(f'正在初始化舵机 {servo_id}...')
            
            # 测试连接
            scs_model_number, scs_comm_result, scs_error = self.packetHandler.ping(
                self.portHandler, servo_id)
                
            if scs_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Communication error with servo {servo_id}: {self.packetHandler.getTxRxResult(scs_comm_result)}')
                continue
            elif scs_error != 0:
                self.get_logger().error(f'Error with servo {servo_id}: {self.packetHandler.getRxPacketError(scs_error)}')
                continue
                
            self.get_logger().info(f'[ID:{servo_id:03d}] ping Succeeded. SCServo model number : {scs_model_number}')
            
            # 设置运行模式为速度模式
            scs_comm_result = self.packetHandler.write1ByteTxOnly(
                self.portHandler, servo_id, ADDR_SCS_OPERATION_MODE, 1)
                
            if scs_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Failed to set operation mode for servo {servo_id}: {self.packetHandler.getTxRxResult(scs_comm_result)}')
                continue
                
            # 等待一下确保模式设置完成
            self.get_clock().sleep_for(Duration(seconds=0.1))
                
            # 验证运行模式
            current_mode, scs_comm_result, scs_error = self.packetHandler.read1ByteTxRx(
                self.portHandler, servo_id, ADDR_SCS_OPERATION_MODE)
                
            if scs_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Failed to read operation mode for servo {servo_id}: {self.packetHandler.getTxRxResult(scs_comm_result)}')
                continue
            elif scs_error != 0:
                self.get_logger().error(f'Error reading operation mode for servo {servo_id}: {self.packetHandler.getRxPacketError(scs_error)}')
                continue
                
            self.get_logger().info(f'舵机 {servo_id} 运行模式: {current_mode}')
            
            # 设置初始速度为0
            scs_comm_result, scs_error = self.packetHandler.write2ByteTxRx(
                self.portHandler, servo_id, ADDR_SCS_GOAL_SPEED, 0)
                
            if scs_comm_result != COMM_SUCCESS:
                self.get_logger().error(f'Failed to set initial speed for servo {servo_id}: {self.packetHandler.getTxRxResult(scs_comm_result)}')
                continue
            elif scs_error != 0:
                self.get_logger().error(f'Error setting initial speed for servo {servo_id}: {self.packetHandler.getRxPacketError(scs_error)}')
                continue
                
            self.get_logger().info(f'舵机 {servo_id} 初始化完成')
            
        self.get_logger().info('所有舵机初始化完成')
        return True
    
    def key_callback(self, msg):
        key = msg.data
        if key in SPEED_MAP:
            speeds = SPEED_MAP[key]
            for servo_id, speed in zip(self.servo_ids, speeds):
                self.set_speed(servo_id, speed)
            self.get_logger().info(f'按键 {key} 对应速度: {speeds}')
    
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
       
    def __del__(self):
        if hasattr(self, 'portHandler'):
            self.portHandler.closePort()

def main(args=None):
    rclpy.init(args=args)
    servo_controller = ServoController()
    rclpy.spin(servo_controller)
    servo_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()