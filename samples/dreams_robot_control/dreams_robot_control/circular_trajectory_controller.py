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
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_odometry = VehicleOdometry()
        self.vehicle_status = VehicleStatus()
        self.start_time = time.time()
        
        # Flight parameters
        self.TARGET_HEIGHT = 5.0  # meters
        self.CIRCLE_DIAMETER = 5.0  # meters
        self.CIRCLE_PERIOD = 30.0  # seconds for one complete circle
        self.HEIGHT_REACHED_THRESHOLD = 0.3  # meters
        self.POSITION_THRESHOLD = 0.3  # meters
        self.MAX_GOTO_VELOCITY = 1.0  # Maximum velocity for moving to start position (m/s)
        
        # Circle center position
        self.CIRCLE_CENTER_X = 5.0  # X coordinate of circle center
        self.CIRCLE_CENTER_Y = 5.0  # Y coordinate of circle center
        
        # State machine
        self.state = "TAKEOFF"  # States: TAKEOFF, GOTO_START, GOTO_CIRCLE_START, ALIGN_YAW, CIRCLE
        
        # Circle parameters
        self.circle_radius = self.CIRCLE_DIAMETER / 2.0
        self.circle_velocity = (2 * math.pi * self.circle_radius) / self.CIRCLE_PERIOD  # m/s
        
        # Control parameters
        self.height_P_gain = 2.0  # Proportional gain for height control
        self.max_vertical_velocity = 2.0  # Maximum vertical velocity (m/s)
        self.takeoff_velocity = 1.5  # Reduced for more stable takeoff

        # Add status flags
        self.offboard_mode_enabled = False
        self.arm_state = False

        # Add initial yaw tracking
        self.initial_yaw = 0.0
        self.initial_yaw_set = False

        # Calculate initial circle point (at 0 degrees)
        self.circle_start_x = self.CIRCLE_CENTER_X + self.circle_radius
        self.circle_start_y = self.CIRCLE_CENTER_Y
        
        # Add yaw alignment threshold
        self.YAW_ALIGNMENT_THRESHOLD = 0.1  # radians (about 5.7 degrees)
        self.MAX_YAW_RATE = 3.0  # radians per second
        self.YAW_RATE_GAIN = 10.0  # Proportional gain for yaw rate control

        # Create a timer to publish control commands
        self.create_timer(0.1, self.timer_callback)  # 10Hz control loop

    def vehicle_odometry_callback(self, msg):
        """Callback function for vehicle odometry data."""
        self.vehicle_odometry = msg
        
        # Capture initial yaw angle if not set
        if not self.initial_yaw_set and hasattr(msg, 'q'):
            # Convert quaternion to Euler angles
            roll, pitch, yaw = self.quaternion_to_euler(msg.q)
            self.initial_yaw = yaw
            self.initial_yaw_set = True

    def vehicle_status_callback(self, msg):
        """Callback function for vehicle status data."""
        self.vehicle_status = msg
        
        # Update status flags
        self.offboard_mode_enabled = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        self.arm_state = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        
        # Log state changes
        self.get_logger().debug(f"Vehicle Status - Offboard: {self.offboard_mode_enabled}, Armed: {self.arm_state}")

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
        msg.position = [float(p) for p in [x, y, z]]  # Position in meters
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
        # self.get_logger().info(f"Current height: {current_height:.2f}m, Target: {self.TARGET_HEIGHT}m")
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

    def is_at_target_position(self, target_x, target_y):
        """Check if drone has reached target position within threshold."""
        current_x = self.vehicle_odometry.position[0]
        current_y = self.vehicle_odometry.position[1]
        distance = math.sqrt((current_x - target_x)**2 + (current_y - target_y)**2)
        return distance < self.POSITION_THRESHOLD

    def calculate_limited_position(self, current_x, current_y, target_x, target_y, max_step):
        """Calculate next position with limited velocity."""
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance <= max_step:
            return target_x, target_y
            
        # Scale the movement to respect max velocity
        scale = max_step / distance
        new_x = current_x + dx * scale
        new_y = current_y + dy * scale
        
        return new_x, new_y

    def quaternion_to_euler(self, q):
        """Convert quaternion to Euler angles (roll, pitch, yaw)."""
        # Extract quaternion components
        w, x, y, z = q[0], q[1], q[2], q[3]
        
        # Calculate yaw (z-axis rotation)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        
        # Calculate pitch (y-axis rotation)
        pitch = math.asin(2.0 * (w * y - z * x))
        
        # Calculate roll (x-axis rotation)
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        
        return roll, pitch, yaw

    def is_yaw_aligned(self, target_yaw, current_yaw):
        """Check if current yaw is aligned with target within threshold."""
        yaw_error = abs(target_yaw - current_yaw)
        # Normalize to [-pi, pi]
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        return abs(yaw_error) < self.YAW_ALIGNMENT_THRESHOLD

    def calculate_limited_yaw(self, current_yaw, target_yaw, max_rate):
        """Calculate next yaw angle with limited rate."""
        # Calculate yaw error
        yaw_error = target_yaw - current_yaw
        
        # Normalize to [-pi, pi]
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
            
        # Calculate yaw rate (proportional to error, limited by max rate)
        yaw_rate = self.YAW_RATE_GAIN * yaw_error
        yaw_rate = max(min(yaw_rate, max_rate), -max_rate)
        
        # Calculate next yaw
        next_yaw = current_yaw + yaw_rate * 0.1  # 0.1 is the control period
        
        return next_yaw

    def timer_callback(self):
        """Timer callback for control loop."""
        # Always publish offboard control mode
        self.publish_offboard_control_mode()

        # Start offboard control and arming sequence after 10 setpoints
        if self.offboard_setpoint_counter >= 10:
            # Check if offboard mode is enabled
            if not self.offboard_mode_enabled:
                self.get_logger().info("Waiting for offboard mode...")
                self.engage_offboard_mode()
            # Only try to arm once offboard mode is confirmed
            elif not self.arm_state:
                self.get_logger().info("Offboard mode confirmed, attempting to arm...")
                self.arm()
                self.start_time = time.time()
            # Only proceed with trajectory once armed
            elif self.arm_state:
                if self.state == "TAKEOFF":
                    if not self.is_at_target_height():
                        self.publish_trajectory_setpoint(
                            x=self.vehicle_odometry.position[0],
                            y=self.vehicle_odometry.position[1],
                            z=-self.TARGET_HEIGHT,
                            yaw=self.initial_yaw  # Keep initial yaw during takeoff
                        )
                    else:
                        self.state = "GOTO_START"
                        self.get_logger().info("Target height reached, moving to circle center")

                elif self.state == "GOTO_START":
                    # Calculate next position with limited velocity
                    next_x, next_y = self.calculate_limited_position(
                        self.vehicle_odometry.position[0],
                        self.vehicle_odometry.position[1],
                        self.CIRCLE_CENTER_X,
                        self.CIRCLE_CENTER_Y,
                        self.MAX_GOTO_VELOCITY
                    )
                    
                    # Move to circle center with limited velocity, keep initial yaw
                    self.publish_trajectory_setpoint(
                        x=next_x,
                        y=next_y,
                        z=-self.TARGET_HEIGHT,
                        yaw=self.initial_yaw  # Keep initial yaw during transition
                    )
                    
                    # Check if we've reached the center position
                    if self.is_at_target_position(self.CIRCLE_CENTER_X, self.CIRCLE_CENTER_Y):
                        self.state = "GOTO_CIRCLE_START"
                        self.get_logger().info("Reached circle center, moving to circle start position")

                elif self.state == "GOTO_CIRCLE_START":
                    # Calculate next position with limited velocity
                    next_x, next_y = self.calculate_limited_position(
                        self.vehicle_odometry.position[0],
                        self.vehicle_odometry.position[1],
                        self.circle_start_x,
                        self.circle_start_y,
                        self.MAX_GOTO_VELOCITY
                    )
                    
                    # Move to circle start position, keep initial yaw
                    self.publish_trajectory_setpoint(
                        x=next_x,
                        y=next_y,
                        z=-self.TARGET_HEIGHT,
                        yaw=self.initial_yaw  # Keep initial yaw
                    )
                    
                    # Check if we've reached the circle start position
                    if self.is_at_target_position(self.circle_start_x, self.circle_start_y):
                        self.state = "ALIGN_YAW"
                        self.get_logger().info("Reached circle start, aligning yaw")

                elif self.state == "ALIGN_YAW":
                    # Calculate desired yaw to face circle center
                    dx = self.CIRCLE_CENTER_X - self.vehicle_odometry.position[0]
                    dy = self.CIRCLE_CENTER_Y - self.vehicle_odometry.position[1]
                    target_yaw = math.atan2(dy, dx)
                    
                    # Get current yaw from odometry
                    _, _, current_yaw = self.quaternion_to_euler(self.vehicle_odometry.q)
                    
                    # Calculate next yaw with rate limiting
                    next_yaw = self.calculate_limited_yaw(
                        current_yaw,
                        target_yaw,
                        self.MAX_YAW_RATE
                    )
                    
                    # Hold position while aligning yaw
                    self.publish_trajectory_setpoint(
                        x=self.circle_start_x,
                        y=self.circle_start_y,
                        z=-self.TARGET_HEIGHT,
                        yaw=next_yaw  # Use rate-limited yaw
                    )
                    
                    # Check if yaw is aligned
                    if self.is_yaw_aligned(target_yaw, current_yaw):
                        self.state = "CIRCLE"
                        self.get_logger().info("Yaw aligned, starting circular motion")
                        self.start_time = time.time()
                    else:
                        yaw_error = abs(target_yaw - current_yaw)
                        self.get_logger().info(
                            f"Aligning yaw: current={math.degrees(current_yaw):.1f}°, "
                            f"target={math.degrees(target_yaw):.1f}°, "
                            f"error={math.degrees(yaw_error):.1f}°, "
                            f"rate={math.degrees(self.MAX_YAW_RATE):.1f}°/s"
                        )

                elif self.state == "CIRCLE":
                    t = time.time() - self.start_time
                    # Calculate position relative to circle center
                    x = self.CIRCLE_CENTER_X + self.circle_radius * math.cos((2 * math.pi * t) / self.CIRCLE_PERIOD)
                    y = self.CIRCLE_CENTER_Y + self.circle_radius * math.sin((2 * math.pi * t) / self.CIRCLE_PERIOD)
                    
                    # Calculate yaw to face circle center (only when in CIRCLE state)
                    dx = self.CIRCLE_CENTER_X - x
                    dy = self.CIRCLE_CENTER_Y - y
                    yaw = math.atan2(dy, dx)
                    
                    self.publish_trajectory_setpoint(
                        x=x, 
                        y=y, 
                        z=-self.TARGET_HEIGHT,
                        yaw=yaw  # Start changing yaw during circle
                    )
                    
                    circle_progress = (t % self.CIRCLE_PERIOD) / self.CIRCLE_PERIOD * 100
                    current_height = -self.vehicle_odometry.position[2]
                    current_velocity = math.sqrt(dx*dx + dy*dy)
                    
                    self.get_logger().info(
                        f"Circle: {circle_progress:.1f}%, "
                        f"Height: {current_height:.2f}m, "
                        f"Velocity: {current_velocity:.2f}m/s, "
                        f"Yaw: {math.degrees(yaw):.1f}°"
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