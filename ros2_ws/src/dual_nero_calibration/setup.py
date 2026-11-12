from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'dual_nero_calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='todo@example.com',
    description='ChArUco camera extrinsic calibration framework for dual NERO.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture_charuco = dual_nero_calibration.capture_charuco:main',
            'capture_chessboard = dual_nero_calibration.capture_charuco:main',
            'capture_manual = dual_nero_calibration.capture_manual:main',
            'capture_auto = dual_nero_calibration.capture_auto:main',
            'extract_from_bag = dual_nero_calibration.extract_from_bag:main',
            'solve_extrinsics = dual_nero_calibration.solve_extrinsics:main',
            'publish_extrinsic = dual_nero_calibration.publish_extrinsic:main',
            'calibrate_gripper_tcp = dual_nero_calibration.calibrate_gripper_tcp:main',
            'calibrate_gripper_opening = dual_nero_calibration.calibrate_gripper_opening:main',
        ],
    },
)
