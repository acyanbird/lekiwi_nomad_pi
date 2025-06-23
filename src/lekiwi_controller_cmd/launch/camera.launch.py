from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            parameters=[
                {'time_per_frame': 0.1},
                {'pixel_format': 'YUYV'}
            ]
        )
    ])