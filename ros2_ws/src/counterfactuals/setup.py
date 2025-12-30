import os
from setuptools import setup
from glob import glob

package_name = 'counterfactuals'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    data_files=[
        # ROS 2 package index
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        # package.xml
        (
            'share/' + package_name,
            ['package.xml']
        ),

        # Launch files
        (
            'share/' + package_name + '/launch',
            glob('launch/*') # glob('launch/*.py')
        ),

        # Worlds
        ('share/' + package_name + '/worlds', glob('worlds/*.wbt')),

        # Textures
        (
            'share/' + package_name + '/worlds/textures',
            glob('worlds/textures/*')
        ),

        # Controllers
        ('share/' + package_name + '/controllers/bmw_x5_controller',  glob('controllers/bmw_x5_controller/*')),
        ('share/' + package_name + '/controllers/supervisor', glob('controllers/supervisor/*')),
        ('share/' + package_name + '/controllers/CitroenCZero_controller',  glob('controllers/CitroenCZero_controller/*')),

    ],

    install_requires=['setuptools', 'pandas', 'scikit-learn', 'numpy'],
    zip_safe=True,
    maintainer='Héctor Avilés',
    maintainer_email='havilesa@upv.edu.mx',
    description='Self-driving car Webots + ROS2 package',
    license='Apache License 2.0',
    tests_require=['pytest'],

    entry_points={
      'console_scripts': [
         'lane_detector_canny_hough = counterfactuals.lane_detector_canny_hough:main',
         'obstacle_detector = counterfactuals.obstacle_detector:main',
         'behaviors = counterfactuals.behaviors:main',
         'lane_identification = counterfactuals.lane_identification:main',
         'goal_reached = counterfactuals.goal_reached:main',
         'success = counterfactuals.success:main',
         'stop = counterfactuals.stop:main',
         'latent_collision_detector=counterfactuals.latent_collision_detector:main',
         'action_policy = counterfactuals.action_policy:main'  
      ],
    },
)

