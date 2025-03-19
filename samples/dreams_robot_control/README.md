# Circular Trajectory Controller

This ROS 2 package provides a controller for executing a circular trajectory using PX4 in offboard mode. The controller is designed to work with drones or other vehicles that support offboard control via ROS 2.

## Prerequisites

- **ROS 2 Jazzy**: Ensure you have ROS 2 Jazzy installed and sourced.
- **XRCE DDS**: Install and configure XRCE DDS for communication between ROS 2 and PX4 following the instructions [here](https://docs.px4.io/main/en/middleware/uxrce_dds.html).
- **QGroundControl**: Download and install from [here](http://qgroundcontrol.com/)

## Installation

1. ### Create Symlink for this package to your ROS2 Workspace
   ```bash
   # Create symlink in your ROS2 workspace
   cd ~/ros2_ws/src
   ln -s ~/RAS-SES-598-Space-Robotics-and-AI/samples/dreams_robot_control .
   ```

2. ### Setup the dependency package
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/PX4/PX4-Autopilot.git
   cd ~/ros2_ws
   bash ./src/PX4-Autopilot/Tools/copy_px4_ros_packages.sh .
   ```

3. **Build the Package**: Navigate to your workspace and build the package.
   ```bash
   cd ~/ros2_ws
   colcon build --symlink-install
   #  Note that building px4_msgs might take a bit longer.
   ```

3. **Source the Workspace**: Source the setup file to overlay the workspace.
   ```bash
   source install/local_setup.bash
   ```

## Running the Circular Trajectory Controller

1. **Launch the Simulation**: Start the PX4 simulation environment (e.g., using Gazebo or SITL).
   ```bash
   cd ~/ros2_ws/src/PX4-Autopilot/
   make px4_sitl gazebo
   ```
2. **Launch the XRCE DDS Agent**: Ensure that the XRCE DDS agent is running and configured to communicate with your PX4 setup.
   ```bash
   MicroXRCEAgent udp4 -p 8888
   ```

2. **Configure Vehicle**: 
   - Open QGroundControl
   - Connect to your vehicle
   - Set up the necessary parameters for offboard control

3. **Run the Controller**: Execute the circular trajectory controller node.
   ```bash
   ros2 run dreams_robot_control circular_trajectory_controller
   ```

4. **Monitor the System**: Use ROS 2 tools and QGroundControl to monitor the system state and trajectory execution.
   ```bash
   ros2 topic echo /fmu/in/trajectory_setpoint
   ```

## Customizing the Trajectory

- **Circle Diameter**: Adjust the circle diameter by modifying the `CIRCLE_DIAMETER` parameter in the controller code.
- **Circle Period**: Change the time it takes to complete one circle by setting the `CIRCLE_PERIOD` parameter.

## Troubleshooting

- **Connection Issues**: Ensure that XRCE DDS is correctly configured and that the agent is running.
- **Offboard Mode**: Verify that the vehicle is in offboard mode and that the controller is publishing setpoints.
- **QGroundControl Issues**: Check USB connections and verify your vehicle is properly connected.

## Contributing

Feel free to submit issues and pull requests for improvements.

## License

This work is licensed under the Apache License 2.0. 