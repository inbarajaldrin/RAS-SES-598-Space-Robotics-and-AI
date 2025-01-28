# Circular Trajectory Controller

This ROS 2 package provides a controller for executing a circular trajectory using PX4 in offboard mode. The controller is designed to work with drones or other vehicles that support offboard control via ROS 2.

## Prerequisites

- **ROS 2 Jazzy**: Ensure you have ROS 2 Jazzy installed and sourced.
- **PX4 Autopilot**: Set up PX4 on your vehicle with offboard control enabled.
- **XRCE DDS**: Install and configure XRCE DDS for communication between ROS 2 and PX4.

## Installation

1. **Clone the Repository**: Clone this repository into your ROS 2 workspace.
   ```bash
   cd ~/ros2_ws/src
   git clone <repository_url>
   ```

2. **Build the Package**: Navigate to your workspace and build the package.
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   ```

3. **Source the Workspace**: Source the setup file to overlay the workspace.
   ```bash
   source install/setup.bash
   ```

## Setting Up XRCE DDS

1. **Install XRCE DDS**: Follow the installation instructions for XRCE DDS from the [official documentation](https://github.com/eProsima/Micro-XRCE-DDS).

2. **Configure XRCE DDS**: Ensure that the XRCE DDS agent is running and configured to communicate with your PX4 setup.
   ```bash
   MicroXRCEAgent udp4 -p 8888
   ```

## Running the Circular Trajectory Controller

1. **Launch the Simulation**: Start the PX4 simulation environment (e.g., using Gazebo or SITL).

2. **Run the Controller**: Execute the circular trajectory controller node.
   ```bash
   ros2 run dreams_robot_control circular_trajectory_controller
   ```

3. **Monitor the System**: Use ROS 2 tools to monitor the system state and trajectory execution.
   ```bash
   ros2 topic echo /fmu/in/trajectory_setpoint
   ```

## Customizing the Trajectory

- **Circle Diameter**: Adjust the circle diameter by modifying the `CIRCLE_DIAMETER` parameter in the controller code.
- **Circle Period**: Change the time it takes to complete one circle by setting the `CIRCLE_PERIOD` parameter.

## Troubleshooting

- **Connection Issues**: Ensure that XRCE DDS is correctly configured and that the agent is running.
- **Offboard Mode**: Verify that the vehicle is in offboard mode and that the controller is publishing setpoints.

## Contributing

Feel free to submit issues and pull requests for improvements.

## License

This work is licensed under the Apache License 2.0. 