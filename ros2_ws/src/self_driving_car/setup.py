from setuptools import setup
import os
from glob import glob

package_name = 'self_driving_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ROS 2 package index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # Launch files
        ('share/' + package_name + '/launch', 
         glob('launch/*.py')),

        # World files
        ('share/' + package_name + '/worlds', 
         glob('worlds/*.wbt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Héctor Avilés',
    maintainer_email='havilesa@upv.edu.mx',
    description='Self-driving car Webots + ROS2 package',
    license='Apache License 2.0',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'lane_detector_canny_hough = self_driving_car.lane_detector_canny_hough:main',
            'obstacle_detector = self_driving_car.obstacle_detector:main',
            'behaviors = self_driving_car.behaviors:main',
            'lane_identification = self_driving_car.lane_identification:main',
            'goal_reached = self_driving_car.goal_reached:main',
            'success = self_driving_car.success:main',
            'stop = self_driving_car.stop:main',
            'supervisor_opposite = self_driving_car.supervisor_opposite:main',
            'supervisor_transverse = self_driving_car.supervisor_transverse:main',            
            # Add any other Python nodes here
        ],
    },
)

