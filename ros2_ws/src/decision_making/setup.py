import os
from setuptools import setup
from glob import glob

package_name = 'decision_making'

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
        
        # counterfactuals model subdirectory
        (
            os.path.join(
                'share',
                package_name,
                'decision_making',
                'counterfactuals_model'
            ),
            glob('decision_making/counterfactuals_model/*')
        ),

    ],

    install_requires=['setuptools', 'pandas', 'scikit-learn', 'numpy'],
    zip_safe=True,
    maintainer='Héctor Avilés',
    maintainer_email='havilesa@upv.edu.mx',
    description='Decision making module for the self-driving car',
    license='Apache License 2.0',
    tests_require=['pytest'],

    entry_points={
      'console_scripts': [
         'action_policy = decision_making.action_policy:main',
         'counterfactuals = decision_making.counterfactuals:main'  
      ],
    },
)

