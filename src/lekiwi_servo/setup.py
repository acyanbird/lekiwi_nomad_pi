from setuptools import find_packages, setup

package_name = 'lekiwi_servo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='acy',
    maintainer_email='acy@todo.todo',
    description='ROS2 package for controlling Feetech servos',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'servo_controller = lekiwi_servo.servo_controller:main',
            'servo_teleop = lekiwi_servo.servo_teleop:main'
        ],
    },
)
