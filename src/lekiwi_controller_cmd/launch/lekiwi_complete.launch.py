import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # --- V4L2 Camera Launch Arguments (New Section) ---
    # These arguments allow you to easily configure the camera from the command line.
    v4l2_video_device_arg = DeclareLaunchArgument(
        'v4l2_video_device',
        default_value='/dev/video0', # Default camera device path
        description='Path to the V4L2 video device (e.g., /dev/video0)'
    )
    v4l2_image_width_arg = DeclareLaunchArgument(
        'v4l2_image_width',
        default_value='640',
        description='Desired image width for V4L2 camera'
    )
    v4l2_image_height_arg = DeclareLaunchArgument(
        'v4l2_image_height',
        default_value='480',
        description='Desired image height for V4L2 camera'
    )
    v4l2_pixel_format_arg = DeclareLaunchArgument(
        'v4l2_pixel_format',
        default_value='MJPG', # MJPG is often efficient for USB cameras
        description='Pixel format for V4L2 camera (e.g., YUYV, MJPG)'
    )
    v4l2_framerate_arg = DeclareLaunchArgument(
        'v4l2_framerate',
        default_value='30.0',
        description='Desired framerate for V4L2 camera'
    )
    v4l2_frame_id_arg = DeclareLaunchArgument(
        'v4l2_frame_id',
        default_value='v4l2_camera_link',
        description='Frame ID for the V4L2 camera messages'
    )
    v4l2_camera_info_url_arg = DeclareLaunchArgument(
        'v4l2_camera_info_url',
        # !!! IMPORTANT: Replace 'YOUR_USER' with your actual Linux username !!!
        # E.g., 'file:///home/acy/.ros/camera_info/integrated_webcam_hd:_integrate.yaml'
        # If you don't have a calibration file, the node will warn but still run.
        default_value='file:///home/YOUR_USER/.ros/camera_info/integrated_webcam_hd:_integrate.yaml',
        description='URL to the V4L2 camera calibration file'
    )
    # --- End V4L2 Camera Launch Arguments ---

    return LaunchDescription([
        # --- Declare V4L2 Camera Arguments ---
        v4l2_video_device_arg,
        v4l2_image_width_arg,
        v4l2_image_height_arg,
        v4l2_pixel_format_arg,
        v4l2_framerate_arg,
        v4l2_frame_id_arg,
        v4l2_camera_info_url_arg,
        # --- End Declare V4L2 Camera Arguments ---

        # Original Camera Node (if you uncomment it later)
        # Node(
        #     package='lekiwi_camera',
        #     executable='node',
        #     name='camera_node',
        #     output='screen',
        #     parameters=[{
        #         'device_id': 0
        #     }]
        # ),
        
        # New V4L2 Camera Node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node', # Give it a unique name
            output='screen', # Print node output to console
            parameters=[{
                'video_device': LaunchConfiguration('v4l2_video_device'),
                'image_size': [LaunchConfiguration('v4l2_image_width'), LaunchConfiguration('v4l2_image_height')],
                'pixel_format': LaunchConfiguration('v4l2_pixel_format'),
                'framerate': LaunchConfiguration('v4l2_framerate'),
                'frame_id': LaunchConfiguration('v4l2_frame_id'),
                'camera_info_url': LaunchConfiguration('v4l2_camera_info_url'),
                # 'use_compression': True # Uncomment if you want compressed images (e.g., /image_raw/compressed)
            }]
        ),
        
        # Start custom Kiwi drive controller node
        Node(
            package='lekiwi_controller_cmd',
            executable='listen_controller',
            name='custom_kiwi_drive_controller',
            output='screen'
        ),
        
        # Start servo controller node
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
        
        # Start odometry node
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
        
        # Start keyboard control node (optional)
        # Node(
        #     package='lekiwi_controller_cmd',
        #     executable='keybroad',
        #     name='servo_teleop',
        #     output='screen'
        # ),
    ])