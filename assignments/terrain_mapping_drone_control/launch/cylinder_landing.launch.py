#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Generate launch description for cylinder landing mission."""
    
    # Get the package share directory
    pkg_share = get_package_share_directory('terrain_mapping_drone_control')
        
    # Set Gazebo model and resource paths
    gz_model_path = os.path.join(pkg_share, 'models')

    # # Set initial drone pose
    os.environ['PX4_GZ_MODEL_POSE'] = "0,0,0.1,0,0,0"
    
    # Add launch argument for PX4-Autopilot path
    px4_autopilot_path = LaunchConfiguration('px4_autopilot_path')
    
    # Launch PX4 SITL with x500_depth
    px4_sitl = ExecuteProcess(
        cmd=['make', 'px4_sitl', 'gz_x500_depth_mono'],
        cwd=px4_autopilot_path,
        output='screen'
    )
    
    # Spawn the first cylinder (front, full height)
    spawn_cylinder_front = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder', 'model.sdf'),
            '-name', 'cylinder_front',
            '-x', '5',     # 5 meters in front of the drone
            '-y', '0',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-scale', '1 1 1',  # normal scale
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Spawn the second cylinder (behind, 7m height)
    spawn_cylinder_back = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(gz_model_path, 'cylinder_short', 'model.sdf'),
            '-name', 'cylinder_back',
            '-x', '-5',    # 5 meters behind the drone
            '-y', '0',     # centered on y-axis
            '-z', '0',     # at ground level
            '-R', '0',     # no roll
            '-P', '0',     # no pitch
            '-Y', '0',     # no yaw
            '-static'      # ensure it's static
        ],
        output='screen'
    )

    # Bridge node for camera and odometry
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        parameters=[{
            'use_sim_time': True,
        }],
        arguments=[
            # Front RGB Camera
            '/rgb_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            
            # Front Depth Camera
            '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            # '/depth_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            
            # Down Mono Camera
            '/mono_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/mono_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            
            # Clock and Odometry
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # '/model/x500_depth_mono_0/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.OdometryWithCovariance',
        ],
        remappings=[
            # Front RGB Camera remappings
            ('/rgb_camera', '/drone/front_rgb'),
            ('/rgb_camera/camera_info', '/drone/front_rgb/camera_info'),
            
            # Front Depth Camera remappings
            ('/depth_camera', '/drone/front_depth'),
            # ('/depth_camera/depth_image', '/drone/front_depth/depth'),
            ('/depth_camera/points', '/drone/front_depth/points'),
            ('/camera_info', '/drone/front_depth/camera_info'),
            
            # Down Mono Camera remappings
            ('/mono_camera', '/drone/down_mono'),
            ('/mono_camera/camera_info', '/drone/down_mono/camera_info'),
            
            # Odometry remapping
            # ('/model/x500_depth_mono_0/odometry_with_covariance', '/fmu/out/vehicle_odometry'),
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'px4_autopilot_path',
            default_value=os.environ.get('HOME', '/home/' + os.environ.get('USER', 'user')) + '/PX4-Autopilot',
            description='Path to PX4-Autopilot directory'),
        px4_sitl,
        TimerAction(
            period=2.0,
            actions=[spawn_cylinder_front]
        ),
        TimerAction(
            period=2.5,
            actions=[spawn_cylinder_back]
        ),
        TimerAction(
            period=3.0,
            actions=[bridge]
        )
    ]) 