import os
from setuptools import setup
from glob import glob

package_name = 'simple'

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
        
        # Supervisor
        ('share/' + package_name + '/controllers/supervisor', glob('controllers/supervisor/*')),        

    ],

    install_requires=['setuptools', 'pandas', 'scikit-learn', 'numpy'],
    zip_safe=True,
    maintainer='Héctor Avilés',
    maintainer_email='havilesa@upv.edu.mx',
    description='Simple Webots + ROS2 package',
    license='Apache License 2.0',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
        ],
    },
)

