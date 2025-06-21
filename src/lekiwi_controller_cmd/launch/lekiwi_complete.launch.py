from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动相机节点
        # Node(
        #     package='lekiwi_camera',
        #     executable='node',
        #     name='camera_node',
        #     output='screen',
        #     parameters=[{
        #         'device_id': 0
        #     }]
        # ),
        Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node', # 给节点一个名字
        output='screen', # 将节点的标准输出打印到控制  
        ),      
        # 启动自定义 Kiwi 驱动控制器节点
        Node(
            package='lekiwi_controller_cmd',
            executable='listen_controller',
            name='custom_kiwi_drive_controller',
            output='screen'
        ),
        
        # 启动舵机控制器节点
        Node(
            package='lekiwi_controller_cmd',
            executable='servo',
            name='servo_controller',
            output='screen',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baudrate': 1000000
            }]
        ),
        
        # 启动里程计节点
        # Node(
        #     package='lekiwi_odom',
        #     executable='omni_wheel_odom_node',
        #     name='omni_wheel_odom_node',
        #     output='screen',
        #     parameters=[{
        #         'wheel_radius': 0.073,
        #         'robot_radius': 0.15,
        #         # 'wheel_angle_7': 3.14159,  # 180度 - 舵机7向后
        #         # 'wheel_angle_8': 1.57080,  # 90度 - 舵机8向右
        #         # 'wheel_angle_9': 0.0,      # 0度 - 舵机9向前
        #         'ticks_per_revolution': 4095,
        #         'publish_rate': 10.0
        #     }]
        # ),
        
        # #启动键盘控制节点（可选）
        # Node(
        #     package='lekiwi_controller_cmd',
        #     executable='keybroad',
        #     name='servo_teleop',
        #     output='screen'
        # ),
    ]) 