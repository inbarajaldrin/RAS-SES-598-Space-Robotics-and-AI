#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Generate launch description for terrain mapping with camera bridge."""
    
    # Get the package share directory
    pkg_share = get_package_share_directory('terrain_mapping_drone_control')
    
    # Get paths
    model_path = os.path.join(pkg_share, 'models')
    config_path = os.path.join(pkg_share, 'config')
    
    # Set Gazebo model and resource paths
    gz_model_path = os.path.join(pkg_share, 'models')
    if 'GZ_SIM_MODEL_PATH' in os.environ:
        os.environ['GZ_SIM_MODEL_PATH'] += os.pathsep + gz_model_path
    else:
        os.environ['GZ_SIM_MODEL_PATH'] = gz_model_path

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + gz_model_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = gz_model_path

    # Set initial drone pose (x y z roll pitch yaw)
    os.environ['PX4_GZ_MODEL_POSE'] = '5 5 1.2 0 0 0'
    
    # Launch PX4 SITL with x500_gimbal
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500_gimbal'],
        cwd='/home/jdas/PX4-Autopilot',
        output='screen'
    )
    
    # Spawn the terrain model
    spawn_terrain = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(model_path, 'terrain', 'model.sdf'),
            '-name', 'terrain',
            '-x', '0',
            '-y', '0',
            '-z', '0',     # 5 meters below ground level
            '-R', '1.5708', # Roll (90 degrees)
            '-P', '0',      # Pitch
            '-Y', '1.5708'  # Yaw (90 degrees counterclockwise)
        ],
        output='screen'
    )
    
    # Bridge node for gimbal and camera topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        arguments=[
            # Gimbal control topics
            '/model/x500_gimbal_0/command/gimbal_roll@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/x500_gimbal_0/command/gimbal_pitch@std_msgs/msg/Float64@gz.msgs.Double',
            '/model/x500_gimbal_0/command/gimbal_yaw@std_msgs/msg/Float64@gz.msgs.Double',
            # Camera topics
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            # Removed problematic PointCloud topic
        ],
        remappings=[
            ('/camera', '/drone_camera'),
            ('/depth_camera', '/drone_depth_camera'),
            ('/camera_info', '/drone_camera_info')
        ],
        output='screen'
    )

    # Add delays to ensure proper startup sequence
    delayed_terrain = TimerAction(
        period=2.0,  # 2 second delay
        actions=[spawn_terrain]
    )

    delayed_bridge = TimerAction(
        period=5.0,  # 5 second delay
        actions=[bridge]
    )

    return LaunchDescription([
        px4_sitl,
        delayed_terrain,
        delayed_bridge
    ])
