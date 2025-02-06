from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    pkg_share = FindPackageShare('cart_pole_optimal_control').find('cart_pole_optimal_control')
    urdf_model_path = os.path.join(pkg_share, 'models', 'cart_pole', 'model.urdf')
    
    return LaunchDescription([
        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['cat ', urdf_model_path])}]
        ),

        # Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', 'empty.sdf'],
            output='screen'
        ),

        # Direct topic bridges
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge',
            output='screen',
            arguments=[
                # Cart force command (ROS -> Gazebo)
                '/model/cart_pole/joint/cart_to_base/cmd_force@std_msgs/msg/Float64]gz.msgs.Double',
                # Joint states (Gazebo -> ROS)
                '/world/empty/model/cart_pole/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
                # Clock (Gazebo -> ROS)
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
            ],
        ),

        # Spawn robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
                '-name', 'cart_pole',
                '-allow_renaming', 'true'
            ],
            output='screen'
        ),

        # Controller
        Node(
            package='cart_pole_optimal_control',
            executable='lqr_controller',
            name='lqr_controller',
            output='screen'
        ),

        # Earthquake Force Generator
        Node(
            package='cart_pole_optimal_control',
            executable='earthquake_force_generator',
            name='earthquake_force_generator',
            output='screen',
            parameters=[{
                'base_amplitude': 5.0,  # Base force amplitude in Newtons
                'frequency_range': [0.5, 2.0],  # Frequency range in Hz
                'update_rate': 50.0  # Update rate in Hz
            }]
        )
    ]) 