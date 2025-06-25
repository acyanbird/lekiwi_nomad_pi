#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import math

def generate_launch_description():
    # 声明启动参数 - 全向三轮机器人
    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.05',
        description='轮子半径 (米)'
    )
    
    robot_radius_arg = DeclareLaunchArgument(
        'robot_radius',
        default_value='0.15',
        description='机器人半径 (轮子到中心的距离，米)'
    )
    
    wheel_angle_1_arg = DeclareLaunchArgument(
        'wheel_angle_1',
        default_value='0.0',
        description='第一个轮子的角度 (弧度)'
    )
    
    wheel_angle_2_arg = DeclareLaunchArgument(
        'wheel_angle_2',
        default_value=str(2.0 * math.pi / 3.0),
        description='第二个轮子的角度 (弧度)'
    )
    
    wheel_angle_3_arg = DeclareLaunchArgument(
        'wheel_angle_3',
        default_value=str(4.0 * math.pi / 3.0),
        description='第三个轮子的角度 (弧度)'
    )
    
    ticks_per_revolution_arg = DeclareLaunchArgument(
        'ticks_per_revolution',
        default_value='360',
        description='每转编码器脉冲数'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='发布频率 (Hz)'
    )
    
    # 创建全向三轮 odom 节点
    odom_node = Node(
        package='lekiwi_odom',
        executable='omni_wheel_odom_node',
        name='omni_wheel_odom_node',
        output='screen',
        parameters=[{
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'robot_radius': LaunchConfiguration('robot_radius'),
            'wheel_angle_1': LaunchConfiguration('wheel_angle_1'),
            'wheel_angle_2': LaunchConfiguration('wheel_angle_2'),
            'wheel_angle_3': LaunchConfiguration('wheel_angle_3'),
            'ticks_per_revolution': LaunchConfiguration('ticks_per_revolution'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
        remappings=[
            ('odom', '/odom'),
            ('joint_states', '/joint_states'),
        ]
    )
    
    return LaunchDescription([
        wheel_radius_arg,
        robot_radius_arg,
        wheel_angle_1_arg,
        wheel_angle_2_arg,
        wheel_angle_3_arg,
        ticks_per_revolution_arg,
        publish_rate_arg,
        odom_node,
    ]) 