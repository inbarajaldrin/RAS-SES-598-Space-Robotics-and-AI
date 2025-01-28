from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('cart_pole_optimal_control')
    
    return LaunchDescription([
        # Start the cart pole controller node
        Node(
            package='cart_pole_optimal_control',
            executable='cart_pole_lqr',
            name='cart_pole_lqr',
            output='screen',
            parameters=[{
                'mass_cart': 1.0,
                'mass_pole': 0.1,
                'pole_length': 1.0,
                'gravity': 9.81,
                'Q_x': 1.0,
                'Q_theta': 10.0,
                'R': 1.0,
            }]
        ),
        
        # Start RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(package_dir, 'config', 'cart_pole.rviz')],
        ),
    ]) 