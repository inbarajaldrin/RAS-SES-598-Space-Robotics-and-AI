#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleControlMode, OffboardControlMode, TrajectorySetpoint, VehicleOdometry, VehicleStatus
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray

class SimpleTestNode(Node):
    def __init__(self):
        super().__init__('simple_test_node')
        
        # Configure QoS profile for PX4 communication
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Publishers
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        
        # Subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', 
            self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)
            
        # Subscribe to geometry tracker outputs
        self.cylinder_pose_subscriber = self.create_subscription(
            Point, '/geometry/cylinder_center',
            self.cylinder_pose_callback, 10)
        self.cylinder_info_subscriber = self.create_subscription(
            Float32MultiArray, '/geometry/cylinder_info',
            self.cylinder_info_callback, 10)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_status = VehicleStatus()
        self.cylinder_position = None
        self.cylinder_info = None
        
        # Flight parameters
        self.TARGET_HEIGHT = 1.0  # meters
        self.POSITION_THRESHOLD = 0.1  # meters
        
        # State machine
        self.state = "TAKEOFF"  # States: TAKEOFF, LAND
        
        # Create a timer to publish control commands
        self.create_timer(0.1, self.control_loop)  # 10Hz control loop

    def vehicle_odometry_callback(self, msg):
        """Store vehicle position from odometry."""
        self.vehicle_odometry = msg

    def vehicle_status_callback(self, msg):
        """Store vehicle status."""
        self.vehicle_status = msg

    def arm(self):
        """Send arm command."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Offboard mode command sent")

    def publish_offboard_control_mode(self):
        """Publish offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publish vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z, yaw):
        """Publish trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def is_at_target_height(self):
        """Check if the drone has reached the target height."""
        try:
            current_height = -self.vehicle_odometry.position[2]  # Convert NED to altitude
            return abs(current_height - self.TARGET_HEIGHT) < self.POSITION_THRESHOLD
        except (IndexError, AttributeError):
            return False

    def cylinder_pose_callback(self, msg):
        """Store cylinder position from geometry tracker."""
        self.cylinder_position = msg
        if self.offboard_setpoint_counter % 10 == 0:  # Log every second
            self.get_logger().info(
                f'Cylinder at pixel ({msg.x:.1f}, {msg.y:.1f}) depth {msg.z:.2f}m'
            )

    def cylinder_info_callback(self, msg):
        """Store cylinder information from geometry tracker."""
        self.cylinder_info = msg.data  # [width, height, angle, confidence]
        if self.offboard_setpoint_counter % 10 == 0:  # Log every second
            self.get_logger().info(
                f'Cylinder size: {msg.data[0]:.1f}x{msg.data[1]:.1f} '
                f'angle: {msg.data[2]:.1f}° confidence: {msg.data[3]:.2f}'
            )

    def control_loop(self):
        """Timer callback for control loop."""
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.get_logger().info("Vehicle armed and offboard mode enabled")

        self.publish_offboard_control_mode()

        try:
            current_height = -self.vehicle_odometry.position[2]
        except (IndexError, AttributeError):
            current_height = 0.0

        if self.state == "TAKEOFF":
            # Take off to target height
            self.publish_trajectory_setpoint(
                x=0.0,
                y=0.0,
                z=-self.TARGET_HEIGHT,  # Negative because PX4 uses NED
                yaw=0.0
            )
            
            # Log current height every second
            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info(f"Taking off... Current height: {current_height:.2f}m")
            
            # Check if we've reached target height
            if self.is_at_target_height():
                self.state = "LAND"
                self.get_logger().info(f"Reached target height of {self.TARGET_HEIGHT}m, beginning landing")

        elif self.state == "LAND":
            # Land by going to height 0
            self.publish_trajectory_setpoint(
                x=0.0,
                y=0.0,
                z=0.0,  # Land at ground level
                yaw=0.0
            )
            
            # Log current height every second
            if self.offboard_setpoint_counter % 10 == 0:
                self.get_logger().info(f"Landing... Current height: {current_height:.2f}m")

        self.offboard_setpoint_counter += 1

def main():
    rclpy.init()
    node = SimpleTestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 