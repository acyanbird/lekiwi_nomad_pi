from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'lekiwi_controller_cmd'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='acy',
    maintainer_email='acyanbird@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo = lekiwi_controller_cmd.servo:main',
            'keybroad = lekiwi_controller_cmd.keybroad:main',
            'listen_controller = lekiwi_controller_cmd.listen_controller:main',
        ],
    },
)
