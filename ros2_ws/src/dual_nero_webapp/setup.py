from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'dual_nero_webapp'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'static'), glob('static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='todo@example.com',
    description='Browser dashboard for the table-mounted dual NERO system.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'webapp_node = dual_nero_webapp.webapp_node:main',
        ],
    },
)
