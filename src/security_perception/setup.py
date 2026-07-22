from setuptools import find_packages, setup

package_name = 'security_perception'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gaurang Chaudhary',
    maintainer_email='gaurang@example.com',
    description='YOLOv8, LiDAR and patrol-zone security anomaly detection.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yolo_node = security_perception.yolo_detection_node:main',
            'event_logger_node = security_perception.event_logger_node:main',
            'gazebo_alert_visualizer = security_perception.gazebo_alert_visualizer:main',
            'sim_anomaly_detector = security_perception.sim_anomaly_detector:main',
        ],
    },
)
