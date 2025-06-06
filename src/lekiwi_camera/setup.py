from setuptools import setup
import os
from glob import glob

package_name = 'lekiwi_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='acy',
    maintainer_email='acyanbird@gmail.com',
    description='ROS2 package for managing V4L2 camera devices',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'node = lekiwi_camera.node:main',
            'camera_node = lekiwi_camera.camera_node:main',
            'test_opencv = lekiwi_camera.test_opencv:main',
        ],
    },
)
