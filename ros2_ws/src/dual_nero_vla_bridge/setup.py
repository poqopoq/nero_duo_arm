from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'dual_nero_vla_bridge'

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
    description='Local observation/action bridge between ROS 2 NERO and LeRobot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vla_bridge = dual_nero_vla_bridge.vla_bridge:main',
            'bimanual_vla_bridge = dual_nero_vla_bridge.bimanual_bridge:main',
        ],
    },
)

