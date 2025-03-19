from setuptools import setup

package_name = 'dreams_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jdas',
    maintainer_email='jnaneshwar.das@gmail.com',
    description='PX4 velocity control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circular_trajectory_controller = dreams_robot_control.circular_trajectory_controller:main',
        ],
    },
)
