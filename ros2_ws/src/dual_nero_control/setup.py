from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'dual_nero_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='todo@example.com',
    description='MoveIt bridge and gripper diagnostics for the dual NERO arms.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'trajectory_bridge = dual_nero_control.trajectory_bridge:main',
            'zqwl_canfd_probe = dual_nero_control.zqwl_canfd_probe:main',
            # Default ROS entry points now drive AGILINK OmniPicker 3 over CAN FD.
            'right_gripper_node = dual_nero_control.omnipicker_node:main',
            'left_gripper_node = dual_nero_control.omnipicker_node:left_main',
            # Keep the former CTAG2F90-D driver available for hardware migration.
            'ctag_right_gripper_node = dual_nero_control.right_gripper_node:main',
            'ctag_left_gripper_node = dual_nero_control.right_gripper_node:left_main',
        ],
    },
)
