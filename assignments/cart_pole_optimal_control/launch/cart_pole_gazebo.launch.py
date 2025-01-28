from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_cart_pole = FindPackageShare('cart_pole_optimal_control')
    
    # Paths to resources
    model_path = PathJoinSubstitution([pkg_cart_pole, 'models'])
    sdf_path = PathJoinSubstitution([model_path, 'cart_pole', 'model.sdf'])
    
    # Start Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )
    
    # Spawn the cart pole
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'cart_pole',
                  '-file', sdf_path],
        output='screen'
    )
    
    # Start the LQR controller
    cart_pole_controller = Node(
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
    )
    
    # Bridge to convert Gazebo topics to ROS topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock bridge
            '/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint state bridge
            '/world/empty/model/cart_pole/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Command bridge for cart only
            '/model/cart_pole/joint/cart_slider/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double'
        ],
        remappings=[
            ('/world/empty/model/cart_pole/joint_state', '/cart_pole/joint_states'),
            ('/model/cart_pole/joint/cart_slider/cmd_vel', '/cart_pole/cart_slider_cmd'),
        ],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        spawn_entity,
        cart_pole_controller,
        bridge
    ]) 