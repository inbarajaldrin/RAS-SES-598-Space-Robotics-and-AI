#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import time

from px4_msgs.msg import VehicleOdometry, OffboardControlMode, VehicleCommand, VehicleStatus, TrajectorySetpoint
from std_msgs.msg import Float64

class SpiralTrajectory(Node):
    """
    A ROS 2 node for executing a spiral descent trajectory with PX4.
    The drone starts from a specified height and performs a descending spiral pattern.
    """
    def __init__(self):
        super().__init__('spiral_trajectory')

        # Configure QoS profile for PX4 communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
            
        # Gimbal control publishers
        self.gimbal_pitch_publisher = self.create_publisher(
            Float64, '/model/x500_gimbal_0/command/gimbal_pitch', 10)
        self.gimbal_roll_publisher = self.create_publisher(
            Float64, '/model/x500_gimbal_0/command/gimbal_roll', 10)
        self.gimbal_yaw_publisher = self.create_publisher(
            Float64, '/model/x500_gimbal_0/command/gimbal_yaw', 10)

        # Subscribers
        self.vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', 
            self.vehicle_odometry_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_status = VehicleStatus()
        self.start_time = time.time()
        
        # Spiral parameters
        self.INITIAL_HEIGHT = 20.0  # meters
        self.SPIRAL_DIAMETER = 35.0  # meters
        self.DESCENT_RATE = 0.5    # m/s
        self.SPIRAL_PERIOD = 10.0   # seconds for one complete revolution
        self.MIN_HEIGHT = 5.0       # minimum height before landing
        self.HEIGHT_REACHED_THRESHOLD = 0.3  # meters
        
        # State machine
        self.state = "TAKEOFF"  # States: TAKEOFF, SPIRAL, LAND
        
        # Create a timer to publish control commands
        self.create_timer(0.1, self.timer_callback)  # 10Hz control loop

    def vehicle_odometry_callback(self, msg):
        """Callback function for vehicle odometry data."""
        self.vehicle_odometry = msg
        self.update_gimbal_angles()

    def vehicle_status_callback(self, msg):
        """Callback function for vehicle status data."""
        self.vehicle_status = msg

    def update_gimbal_angles(self):
        """Update gimbal angles to point at takeoff position (0,0,0)."""
        # Get current drone position
        x = self.vehicle_odometry.position[0]
        y = self.vehicle_odometry.position[1]
        z = self.vehicle_odometry.position[2]  # Note: z is negative when above ground

        # Calculate distance to takeoff point in horizontal plane
        distance_xy = math.sqrt(x*x + y*y)
        
        # Calculate angles
        # Pitch angle (negative to point down)
        pitch_angle = -math.atan2(-z, distance_xy)  # Negative because z is negative when above ground
        
        # Yaw angle (to point at takeoff position)
        yaw_angle = math.atan2(y, x)
        if yaw_angle < 0:
            yaw_angle += 2 * math.pi  # Convert to [0, 2π] range
        
        # Keep roll level
        roll_angle = 0.0

        # Publish gimbal commands
        self.publish_gimbal_angle('pitch', pitch_angle)
        self.publish_gimbal_angle('roll', roll_angle)
        self.publish_gimbal_angle('yaw', yaw_angle)

    def publish_gimbal_angle(self, axis, angle):
        """Publish gimbal angle command."""
        msg = Float64()
        msg.data = float(angle)
        if axis == 'pitch':
            self.gimbal_pitch_publisher.publish(msg)
        elif axis == 'roll':
            self.gimbal_roll_publisher.publish(msg)
        elif axis == 'yaw':
            self.gimbal_yaw_publisher.publish(msg)

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("Disarm command sent")

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Offboard mode command sent")

    def publish_offboard_control_mode(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        """Publish trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publish vehicle command."""
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def is_at_target_height(self):
        """Check if the drone has reached the target height."""
        current_height = -self.vehicle_odometry.position[2]
        return abs(current_height - self.INITIAL_HEIGHT) < self.HEIGHT_REACHED_THRESHOLD

    def calculate_spiral_position(self, time_elapsed):
        """Calculate position along spiral trajectory."""
        # Calculate angular position
        angle = (2 * math.pi * time_elapsed) / self.SPIRAL_PERIOD
        radius = self.SPIRAL_DIAMETER / 2.0
        
        # Calculate position
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        z = -(self.INITIAL_HEIGHT - self.DESCENT_RATE * time_elapsed)
        
        # Calculate yaw to always point towards the center
        yaw = angle + math.pi
        
        return x, y, z, yaw

    def timer_callback(self):
        """Timer callback for control loop."""
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()
            self.start_time = time.time()

        self.publish_offboard_control_mode()

        if self.state == "TAKEOFF":
            if not self.is_at_target_height():
                self.publish_trajectory_setpoint(
                    x=0.0,
                    y=0.0,
                    z=-self.INITIAL_HEIGHT,
                    yaw=0.0
                )
                self.get_logger().info(f"Taking off... Current height: {-self.vehicle_odometry.position[2]:.2f}m")
            else:
                self.state = "SPIRAL"
                self.start_time = time.time()
                self.get_logger().info("Starting spiral descent")

        elif self.state == "SPIRAL":
            time_elapsed = time.time() - self.start_time
            x, y, z, yaw = self.calculate_spiral_position(time_elapsed)
            
            # Check if we've reached minimum height
            if -z <= self.MIN_HEIGHT:
                self.state = "LAND"
                self.get_logger().info("Reached minimum height, preparing to land")
            else:
                self.publish_trajectory_setpoint(x=x, y=y, z=z, yaw=yaw)
                self.get_logger().info(
                    f"Spiral descent... Height: {-z:.2f}m, "
                    f"Position: ({x:.2f}, {y:.2f})"
                )

        elif self.state == "LAND":
            self.publish_trajectory_setpoint(
                x=0.0,
                y=0.0,
                z=0.0,
                yaw=0.0
            )
            self.get_logger().info("Landing...")

        self.offboard_setpoint_counter += 1

def main():
    print('Starting spiral trajectory...')
    rclpy.init()
    
    spiral_node = SpiralTrajectory()
    print('Spiral trajectory node created. Starting control loop...')
    
    try:
        rclpy.spin(spiral_node)
    except KeyboardInterrupt:
        print('Stopping spiral trajectory...')
    finally:
        spiral_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 