#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math
import time

from px4_msgs.msg import VehicleOdometry, VehicleRatesSetpoint, OffboardControlMode, VehicleCommand, VehicleStatus, TrajectorySetpoint

class PX4VelocityController(Node):
    """
    A ROS 2 node for controlling PX4 in velocity mode using offboard control.
    """
    def __init__(self):
        super().__init__('px4_velocity_controller')

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
        
        # Flight parameters
        self.TARGET_HEIGHT = 10.0  # meters
        self.CIRCLE_DIAMETER = 20.0  # meters
        self.CIRCLE_PERIOD = 30.0  # seconds for one complete circle
        self.HEIGHT_REACHED_THRESHOLD = 0.3  # meters
        self.state = "TAKEOFF"  # States: TAKEOFF, CIRCLE
        
        # Circle parameters
        self.circle_radius = self.CIRCLE_DIAMETER / 2.0
        self.circle_velocity = (2 * math.pi * self.circle_radius) / self.CIRCLE_PERIOD  # m/s
        
        # Control parameters
        self.height_P_gain = 2.0  # Proportional gain for height control
        self.max_vertical_velocity = 2.0  # Maximum vertical velocity (m/s)
        self.takeoff_velocity = 1.5  # Reduced for more stable takeoff

        # Create a timer to publish control commands
        self.create_timer(0.1, self.timer_callback)  # 10Hz control loop

    def vehicle_odometry_callback(self, msg):
        """Callback function for vehicle odometry data."""
        self.vehicle_odometry = msg

    def vehicle_status_callback(self, msg):
        """Callback function for vehicle status data."""
        self.vehicle_status = msg

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
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        """Publish trajectory setpoint for position control."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]  # Position in meters
        msg.yaw = yaw  # Yaw in radians
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
        if not hasattr(self.vehicle_odometry, 'position') or len(self.vehicle_odometry.position) < 3:
            self.get_logger().warning("Height data is invalid or not available.")
            return False
        current_height = -self.vehicle_odometry.position[2]  # Convert NED to positive altitude
        self.get_logger().info(f"Current height: {current_height:.2f}m, Target: {self.TARGET_HEIGHT}m")
        return abs(current_height - self.TARGET_HEIGHT) < self.HEIGHT_REACHED_THRESHOLD

    def get_height_error(self):
        """Get the error in height from target."""
        if not hasattr(self.vehicle_odometry, 'position') or len(self.vehicle_odometry.position) < 3:
            self.get_logger().warning("Height data is invalid or not available.")
            return 0.0
        current_height = -self.vehicle_odometry.position[2]  # Convert NED to positive altitude
        height_error = self.TARGET_HEIGHT - current_height
        self.get_logger().info(f"Height error: {height_error:.2f}m")
        return height_error

    def calculate_circle_velocity(self, t):
        """Calculate velocity components for circular motion."""
        # Calculate angular position
        angular_pos = (2 * math.pi * t) / self.CIRCLE_PERIOD
        
        # Calculate velocity components for circular motion
        vx = -self.circle_velocity * math.sin(angular_pos)  # negative sine for clockwise motion
        vy = self.circle_velocity * math.cos(angular_pos)
        
        # Calculate desired yaw (pointing towards center of circle)
        yaw = math.atan2(-vy, -vx)  # Points to circle center
        
        # Scale velocities for smoother motion
        velocity_scale = min(1.0, t / 3.0)  # Ramp up over 3 seconds
        return vx * velocity_scale, vy * velocity_scale, yaw

    def timer_callback(self):
        """Timer callback for control loop."""
        if self.offboard_setpoint_counter == 10:
            # Enable offboard mode after 10 setpoints
            self.engage_offboard_mode()
            self.arm()
            self.start_time = time.time()

        # Publish offboard control mode
        self.publish_offboard_control_mode()

        if self.state == "TAKEOFF":
            # During takeoff, move to target height
            if not self.is_at_target_height():
                self.publish_trajectory_setpoint(
                    x=self.vehicle_odometry.position[0],
                    y=self.vehicle_odometry.position[1],
                    z=-self.TARGET_HEIGHT,  # Target height in NED frame
                    yaw=0.0
                )
                self.get_logger().info(f"Taking off... Current height: {-self.vehicle_odometry.position[2]:.2f}m, Target: {self.TARGET_HEIGHT}m")
            else:
                self.state = "CIRCLE"
                self.get_logger().info("Target height reached, starting circular motion")
                self.start_time = time.time()  # Reset start time for circle motion

        elif self.state == "CIRCLE":
            # Calculate time since starting circle
            t = time.time() - self.start_time
            
            # Calculate position for circular motion
            x = self.circle_radius * math.cos((2 * math.pi * t) / self.CIRCLE_PERIOD)
            y = self.circle_radius * math.sin((2 * math.pi * t) / self.CIRCLE_PERIOD)
            
            # Publish setpoint
            self.publish_trajectory_setpoint(x=x, y=y, z=-self.TARGET_HEIGHT, yaw=0.0)
            
            # Log circle progress and status
            circle_progress = (t % self.CIRCLE_PERIOD) / self.CIRCLE_PERIOD * 100
            current_height = -self.vehicle_odometry.position[2] if hasattr(self.vehicle_odometry, 'position') else 0.0
            current_velocity = math.sqrt(x*x + y*y)
            
            self.get_logger().info(
                f"Circle: {circle_progress:.1f}%, "
                f"Height: {current_height:.2f}m, "
                f"Velocity: {current_velocity:.2f}m/s"
            )

        self.offboard_setpoint_counter += 1

def main(args=None):
    rclpy.init(args=args)
    controller = PX4VelocityController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Stopping controller...')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 