# Terrain Mapping Drone Control

This ROS2 package implements a drone control system for terrain mapping of the Bishop Fault Scarp using ORBSLAM3 and PX4 SITL simulation.

## Overview

The package provides a controller that guides a PX4-powered drone in a lawnmower pattern over the Bishop Fault Scarp area while maintaining a constant height. The drone captures images that are processed by ORBSLAM3 for simultaneous localization and mapping (SLAM).

### Key Features

- Automated takeoff and landing
- Lawnmower pattern trajectory generation for complete area coverage
- Integration with ORBSLAM3 for real-time mapping
- Configurable survey parameters (height, speed, area dimensions)
- Automatic return to launch point after survey completion

## Prerequisites

- ROS2 Humble
- PX4 SITL Simulator
- ORBSLAM3 ROS2 package
- OpenCV
- Python 3.8+

## Installation

1. Clone this package into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository_url>
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select terrain_mapping_drone_control
```

4. Source the workspace:
```bash
source ~/ros2_ws/install/setup.bash
```

## Usage

1. Launch the complete system (PX4 SITL, ORBSLAM3, and the controller):
```bash
ros2 launch terrain_mapping_drone_control terrain_mapping.launch.py
```

### Manual Camera Bridge Setup
If you need to manually bridge the Gazebo camera topics to ROS2, you can use the following commands:
```bash
# Bridge RGB camera
ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image

# Bridge depth camera
ros2 run ros_gz_bridge parameter_bridge /depth_camera@sensor_msgs/msg/Image@gz.msgs.Image

# Bridge camera info
ros2 run ros_gz_bridge parameter_bridge /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo

# Bridge point cloud
ros2 run ros_gz_bridge parameter_bridge /depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloud
```

2. Monitor the progress:
```bash
ros2 topic echo /terrain_mapping_controller/status
```

3. View the ORBSLAM3 visualization:
```bash
ros2 run rqt_image_view rqt_image_view
```

## Configuration

The package behavior can be configured through the following files:

- `config/terrain_mapping_params.yaml`: Main configuration file for flight parameters
- `config/camera_calibration.yaml`: Camera calibration parameters for ORBSLAM3
- `launch/terrain_mapping.launch.py`: Launch file configuration

### Key Parameters

- `mapping_height`: Flight altitude in meters
- `survey_speed`: Drone velocity during mapping
- `area_length`: Length of the survey area (X direction)
- `area_width`: Width of the survey area (Y direction)
- `strip_spacing`: Distance between parallel survey lines

## Output

The system generates the following outputs:

1. ORBSLAM3 trajectory and map data
2. Flight telemetry logs
3. Captured images
4. Generated point cloud of the terrain

Data is saved to the path specified in the configuration file.

## License

Apache License 2.0

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request. 