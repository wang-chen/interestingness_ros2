from setuptools import setup
import os
from glob import glob

package_name = 'interestingness_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('rviz/*.rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bowen',
    maintainer_email='1854152@tongji.edu.cn',
    description='ROS2 Package for interestingness detection using PyTorch',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'live_detector = interestingness_ros2.live_detector:main',
        'interest_marker = interestingness_ros2.interest_marker:main',
        ],
    },
)
