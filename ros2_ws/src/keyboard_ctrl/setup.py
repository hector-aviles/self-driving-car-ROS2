from setuptools import find_packages, setup

package_name = 'keyboard_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
         'keyboard_ctrl = keyboard_ctrl.keyboard_ctrl:main'  
      ],
    },

)
