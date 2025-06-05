from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动 listen_controller 节点
        Node(
            package='lekiwi_controller_cmd',
            executable='listen_controller',
            name='custom_kiwi_drive_controller',
            output='screen'
        ),
        
        # 启动 servo 节点
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
        
        # 启动键盘控制节点
        Node(
            package='lekiwi_controller_cmd',
            executable='keybroad',
            name='servo_teleop',
            output='screen'
        )
    ])