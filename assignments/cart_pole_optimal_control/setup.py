from setuptools import setup
import os
from glob import glob

package_name = 'cart_pole_optimal_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'models/cart_pole'), 
         glob('models/cart_pole/*.urdf')),
        (os.path.join('share', package_name, 'models/cart_pole/meshes'), 
         glob('models/cart_pole/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jnaneshwar Das',
    maintainer_email='jdas5@asu.edu',
    description='ROS2 package for cart-pole control using LQR',
    license='Creative Commons Attribution 4.0',
    entry_points={
        'console_scripts': [
            'lqr_controller = cart_pole_optimal_control.lqr_controller:main',
            'earthquake_force_generator = cart_pole_optimal_control.earthquake_force_generator:main',
            'force_visualizer = cart_pole_optimal_control.force_visualizer:main',
            'state_republisher = cart_pole_optimal_control.state_republisher:main',
        ],
    },
)
