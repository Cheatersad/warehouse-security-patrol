from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'warehouse_patrol'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'tools'), glob('tools/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gaurang Chaudhary',
    maintainer_email='gaurang@example.com',
    description='Warehouse simulation, SLAM, Nav2 and ordered security patrol.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'patrol_manager = warehouse_patrol.patrol_manager:main',
            'demo_anomaly_publisher = warehouse_patrol.demo_anomaly_publisher:main',
            'odom_tf_guard = warehouse_patrol.odom_tf_guard:main',
        ],
    },
)
