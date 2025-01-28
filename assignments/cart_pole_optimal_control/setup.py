from setuptools import setup
import os
from glob import glob

package_name = 'cart_pole_optimal_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), [
            os.path.join('launch', 'cart_pole.launch.py'),
            os.path.join('launch', 'cart_pole_gazebo.launch.py'),
        ]),
        (os.path.join('share', package_name, 'models', 'cart_pole'),
         glob(os.path.join('models', 'cart_pole', '*.sdf'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jdas',
    maintainer_email='jnaneshwar.das@gmail.com',
    description='Cart pole LQR controller with visualization',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'cart_pole_lqr = cart_pole_optimal_control.cart_pole_lqr:main',
        ],
    },
)
