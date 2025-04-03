#!/usr/bin/env python3

import math
import time
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from px4_msgs.msg import VehicleOdometry, OffboardControlMode, VehicleCommand, TrajectorySetpoint, BatteryStatus 
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2
import numpy as np

# For synchronized subscription of RGB + Depth
from message_filters import ApproximateTimeSynchronizer, Subscriber


class CylinderMission(Node):
    def __init__(self):
        super().__init__('cylinder_mission_node')

        # ---------------------------------------------
        # PX4 / Offboard QoS
        # ---------------------------------------------
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ---------------------------------------------
        # Publishers
        # ---------------------------------------------
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile
        )
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )

        # ---------------------------------------------
        # Subscribers
        # ---------------------------------------------
        # Drone odometry
        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odom_cb, qos_profile
        )

        # Camera info (for intrinsics)
        self.caminfo_sub = self.create_subscription(
            CameraInfo, '/drone/front_depth/camera_info',
            self.caminfo_callback, 10
        )

        # Approx time sync for RGB + Depth
        self.rgb_sub = Subscriber(self, Image, '/drone/front_rgb')
        self.depth_sub = Subscriber(self, Image, '/drone/front_depth')
        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.image_callback)

        # ---------------------------------------------
        # Internal State Machine
        # ---------------------------------------------
        # WAIT_INTRINSICS -> ARM_TAKEOFF -> CIRCLE -> SERVO -> HOVER
        # -> LAND -> DISARM -> COMPLETE -> DONE
        self.takeoff_stage = 0  # 0 = vertical, 1 = move to circle start

        self.state = "WAIT_INTRINSICS"
        self.offboard_setpoint_counter = 0

        # Timer for controlling flight logic
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Current drone position: [x, y, z]
        self.position = [0.0, 0.0, 0.0]
        self.bridge = CvBridge()

        # ---------------------------------------------
        # Camera intrinsics
        # ---------------------------------------------
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # ---------------------------------------------
        # Circle flight parameters
        # ---------------------------------------------
        self.circle_radius = 15.0
        self.altitude = -5.0
        self.circle_speed = -0.02  # radians step per iteration
        self.theta = 0.0

        # ---------------------------------------------
        # Cylinder detection and measurement
        # ---------------------------------------------
        self.measured_cylinders = []
        self.points_buffer = []
        self.sample_threshold = 10  # frames to accumulate for stable measurement
        self.desired_distance = 15.0
        self.distance_tolerance = 0.3
        self.hover_start_time = None
        self.servo_start_time = None
        self.min_pixel_area = 5000  # adjust as needed

        # ---------------------------------------------
        # Detection cooldown control
        # ---------------------------------------------
        # We skip detection for 10 seconds after measuring a new cylinder
        self.detection_cooldown_until = 0.0

        # ---------------------------------------------
        # Land on tallest cylinder
        # ---------------------------------------------
        # For ArUco logic
        self.markers = {}
        self.land_target = None

        # ArUco marker pose subscriber (string topic)
        self.marker_pose_sub = self.create_subscription(
            String, '/aruco/marker_pose', self.aruco_cb, 10
        )
        self.aruco_hover_start_time = None

        # ---------------------------------------------
        # Logging mission details
        # ---------------------------------------------
        # Mission timing and energy tracking
        self.start_time = None
        self.battery_percent = None
        self.initial_battery = None

        # For mission battery tracking
        self.battery_at_mission_start = None
        self.battery_at_mission_end = None

        self.battery_sub = self.create_subscription(
            BatteryStatus,
            '/fmu/out/battery_status',
            self.battery_cb,
            qos_profile
        )

    # ---------------------------------------------
    # Battery logging
    # ---------------------------------------------
    def battery_cb(self, msg):
        if not math.isnan(msg.volt_based_soc_estimate):
            # Keep an up-to-date snapshot of battery percentage
            self.battery_percent = msg.volt_based_soc_estimate

    # ---------------------------------------------
    # Callback: Vehicle Odometry
    # ---------------------------------------------
    def odom_cb(self, msg):
        self.position = [msg.position[0], msg.position[1], msg.position[2]]

    # ---------------------------------------------
    # Callback: Camera Info (intrinsics)
    # ---------------------------------------------
    def caminfo_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        self.get_logger().info("Camera intrinsics received.")
        # Unsubscribe after receiving once
        if self.caminfo_sub is not None:
            self.destroy_subscription(self.caminfo_sub)
            self.caminfo_sub = None

    # ---------------------------------------------
    # Callback: Synchronized Image + Depth
    # ---------------------------------------------
    def image_callback(self, rgb_msg, depth_msg):
        # Skip detection during cooldown
        if time.time() < self.detection_cooldown_until:
            return

        # If intrinsics are not known yet, skip
        if self.fx is None or self.fy is None:
            return

        # Convert ROS → OpenCV
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough').astype(np.float32)
        depth[depth == 0] = np.nan

        # Simple color-based segmentation (placeholder logic)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)
        lower_hsv = np.array([0, 0, 110])  # tune to your cylinder color
        upper_hsv = np.array([180, 40, 180])
        color_mask = cv2.inRange(hsv, lower_hsv, upper_hsv) > 0

        # Depth threshold
        depth_mask = np.logical_and(depth > 1.0, depth < 30.0)
        object_mask = np.logical_and(depth_mask, color_mask)

        # Morphological close to reduce noise
        object_mask = cv2.morphologyEx(
            object_mask.astype(np.uint8),
            cv2.MORPH_CLOSE,
            np.ones((5, 5), np.uint8)
        )

        # Contour detection
        contours, _ = cv2.findContours(object_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered = [c for c in contours if cv2.contourArea(c) > self.min_pixel_area]


        # Visualization overlay
        overlay = rgb.copy()

        if len(filtered) > 0:
            # Sort by largest area
            filtered.sort(key=cv2.contourArea, reverse=True)
            contour = filtered[0]
            x, y, w, h = cv2.boundingRect(contour)
            roi = depth[y:y + h, x:x + w]
            roi = roi[np.isfinite(roi)]

            if roi.size > 0:
                # Median depth in bounding box
                Z = float(np.median(roi))
                width_m = (w * Z) / self.fx
                height_m = (h * Z) / self.fy

                self.points_buffer.append((width_m, height_m, Z))

                # Debug bounding box
                cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(
                    overlay,
                    f"{width_m:.2f}m x {height_m:.2f}m",
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1
                )

                # Transition to SERVO if we're in CIRCLE
                if self.state == "CIRCLE":
                    self.get_logger().info("Detected potential cylinder. Switching to SERVO state.")
                    self.state = "SERVO"

        # Show debug windows
        cv2.imshow("RGB Detection", overlay)
        cv2.imshow("Mask", object_mask.astype(np.uint8) * 255)
        cv2.waitKey(1)
    # ---------------------------------------------
    # Aruco detection and transformation to drone coordinates
    # ---------------------------------------------
    def aruco_cb(self, msg):
        import re
        match = re.match(r"Marker (\d+) detected at x:([-\d.]+)m, y:([-\d.]+)m, z:([-\d.]+)m", msg.data)
        if match:
            marker_id = int(match.group(1))
            x = float(match.group(2))
            y = float(match.group(3))
            z = float(match.group(4))
            # Transform to drone frame: x,y,z => y,x,z
            drone_x = y
            drone_y = x
            drone_z = z
            self.markers[marker_id] = (drone_x, drone_y, drone_z)
            self.get_logger().info(f"Updated Marker {marker_id}: x={drone_x}, y={drone_y}, z={drone_z}")

    # ---------------------------------------------
    # Timer Callback: Main State Machine
    # ---------------------------------------------
    def timer_callback(self):
        # Publish offboard control mode each cycle
        self.publish_offboard_control_mode()

        # After ~1s, engage offboard + arm (if intrinsics are loaded)
        if self.offboard_setpoint_counter == 5:
            if self.state != "WAIT_INTRINSICS":
                self.engage_offboard_mode()
                self.arm()

        # State machine

        elif self.state == "WAIT_INTRINSICS":
            if (self.fx is not None) and (self.fy is not None) and (self.battery_percent is not None):
                # store the battery now, one time only
                if self.battery_at_mission_start is None:
                    self.battery_at_mission_start = self.battery_percent
                    self.get_logger().info(f"Locked battery_at_mission_start: {self.battery_at_mission_start:.4f}")

                self.get_logger().info("Intrinsics and battery OK. Moving to ARM_TAKEOFF.")
                self.state = "ARM_TAKEOFF"
                self.start_time = time.time()

        elif self.state == "ARM_TAKEOFF":
            if self.takeoff_stage == 0:
                # Stage 1: Vertical takeoff to (0, 0, -5)
                target = [0.0, 0.0, -5.0]
                self.publish_trajectory_setpoint(*target)

                dx = self.position[0] - target[0]
                dy = self.position[1] - target[1]
                dz = self.position[2] - target[2]
                dist = math.sqrt(dx**2 + dy**2 + dz**2)

                if dist < 0.5:
                    self.get_logger().info("Vertical takeoff complete. Proceeding to circle entry point.")
                    self.takeoff_stage = 1

            elif self.takeoff_stage == 1:
                # Stage 2: Move to (15, 0, -5)
                target = [15.0, 0.0, -5.0]
                self.publish_trajectory_setpoint(*target)

                dx = self.position[0] - target[0]
                dy = self.position[1] - target[1]
                dz = self.position[2] - target[2]
                dist = math.sqrt(dx**2 + dy**2 + dz**2)

                if dist < 0.5:
                    # Set theta based on actual position
                    self.theta = math.atan2(self.position[1], self.position[0])
                    self.get_logger().info("Reached circle entry point. Switching to CIRCLE.")
                    self.state = "CIRCLE"

        elif self.state == "CIRCLE":
            # Circle flight
            x = self.circle_radius * math.cos(self.theta)
            y = self.circle_radius * math.sin(self.theta)
            z = self.altitude
            self.theta += self.circle_speed
            self.publish_trajectory_setpoint(x, y, z)

        elif self.state == "SERVO":
            # Start the timer only once
            if self.servo_start_time is None:
                self.servo_start_time = time.time()

            # Check if we have recent depth data
            current_distance = None
            if len(self.points_buffer) > 0:
                _, _, Z = self.points_buffer[-1]
                current_distance = Z

            if current_distance is None:
                # Timeout logic: give up after 5 seconds
                if time.time() - self.servo_start_time > 5.0:
                    self.get_logger().warn("Object not found within timeout. Returning to CIRCLE.")
                    
                    # Clear stale detection data
                    self.points_buffer.clear()
                    
                    # Reset timer and return to CIRCLE
                    self.servo_start_time = None
                    drone_x, drone_y, _ = self.position
                    self.theta = math.atan2(drone_y, drone_x)
                    self.state = "CIRCLE"

                else:
                    # Keep hovering during search
                    self.publish_trajectory_setpoint(self.position[0], self.position[1], self.altitude)
            else:
                # Move forward/backward to maintain distance
                distance_error = self.desired_distance - current_distance
                drone_x, drone_y, _ = self.position
                gain = 0.5
                dx = distance_error * gain

                target_x = drone_x - dx
                target_y = drone_y
                target_z = self.altitude
                self.publish_trajectory_setpoint(target_x, target_y, target_z)

                # If within tolerance, hover to measure
                if abs(distance_error) < self.distance_tolerance:
                    self.get_logger().info("Reached ~15m from cylinder. Going to HOVER to measure.")
                    self.hover_start_time = time.time()
                    self.servo_start_time = None  # Reset for next time
                    self.state = "HOVER"

        elif self.state == "HOVER":
            # Maintain current position at the hover altitude
            self.publish_trajectory_setpoint(self.position[0], self.position[1], self.altitude)

            # Check if 5 seconds have passed since entering HOVER
            if time.time() - self.hover_start_time >= 7.0:
                self.get_logger().info("7s hover done. Checking measurement.")

                # Check if we collected bounding-box data
                if len(self.points_buffer) > 0:
                    widths, heights, depths = zip(*self.points_buffer)
                    median_w = statistics.median(widths)
                    median_h = statistics.median(heights)
                    self.get_logger().info(
                        f"[Cylinder Dimensions] Width={median_w:.2f} m, Height={median_h:.2f} m"
                    )

                    # Clear buffer for next object
                    self.points_buffer.clear()

                    # Compare with previously measured cylinders
                    dimension_matched = False
                    tolerance = 0.3  # example tolerance
                    for (w_old, h_old) in self.measured_cylinders:
                        if (abs(w_old - median_w) < tolerance) and (abs(h_old - median_h) < tolerance):
                            dimension_matched = True
                            break

                    if dimension_matched:
                        self.get_logger().info(
                            "This cylinder matches a previously seen one. Mission done, landing."
                        )
                        # Go to the LAND state
                        self.state = "ARUCO_HOVER"
                    else:
                        self.get_logger().info(
                            "New cylinder dimension recorded. Resuming circle flight."
                        )
                        self.measured_cylinders.append((median_w, median_h))

                        # OPTIONAL: Set a detection cooldown so the drone
                        # won't detect the same cylinder immediately again
                        self.detection_cooldown_until = time.time() + 6.0

                        # Recalculate theta based on current position
                        drone_x, drone_y, _ = self.position
                        self.theta = math.atan2(drone_y, drone_x)
                        self.get_logger().info(f"Rejoining circle from theta = {self.theta:.2f} rad")

                        # Return to circle state
                        self.state = "CIRCLE"
                else:
                    self.get_logger().warn("No data in points_buffer. Resuming circle anyway.")
                    # Return to circle state
                    self.state = "CIRCLE"

        elif self.state == "ARUCO_HOVER":
            self.publish_trajectory_setpoint(x=0.0, y=0.0, z=-20.0)

            # Start timer once
            if self.aruco_hover_start_time is None:
                if abs(self.position[2] + 20.0) < 0.3:
                    self.aruco_hover_start_time = time.time()
                    self.get_logger().info("Reached hover height. Holding for 5 seconds...")

            # After 5 seconds, transition to next state
            elif time.time() - self.aruco_hover_start_time >= 7.0:
                self.get_logger().info("5s ArUco hover complete. Selecting marker...")
                self.state = "ARUCO_SELECT"

        elif self.state == "ARUCO_SELECT":
            if len(self.markers) >= 2:
                best_marker_id = None
                min_z = float('inf')
                for mid, (mx, my, mz) in self.markers.items():
                    if mz < min_z:
                        min_z = mz
                        best_marker_id = mid
                if best_marker_id is not None:
                    dx, dy, dz = self.markers[best_marker_id]
                    self.land_target = [dx, dy, -abs(20.0 - dz)]
                    self.get_logger().info(
                        f"Selected Marker {best_marker_id} for landing at x={dx:.2f}, y={dy:.2f}, z={-abs(20.0 - dz):.2f}"
                    )
                    self.state = "ARUCO_MOVE"

        elif self.state == "ARUCO_MOVE":
            x, y, z = self.land_target
            self.publish_trajectory_setpoint(x=x, y=y, z=z)
            dist = math.sqrt(
                (self.position[0] - x)**2 +
                (self.position[1] - y)**2 +
                (self.position[2] - z)**2)
            if dist < 0.5:
                self.get_logger().info("Reached marker position. Initiating LAND.")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.state = "ARUCO_LAND"

        elif self.state == "ARUCO_LAND":

            self.get_logger().info("Landed successfully. Disarming...")
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
            )
            self.state = "COMPLETE"

        elif self.state == "COMPLETE":
            self.get_logger().info("Mission complete.")

            if self.battery_at_mission_end is None and self.battery_percent is not None:
                self.battery_at_mission_end = self.battery_percent
                self.get_logger().info(f"Captured battery_at_mission_end: {self.battery_at_mission_end:.4f}")

            if self.start_time is not None:
                mission_duration = time.time() - self.start_time
                self.get_logger().info(f"Mission Duration: {mission_duration:.2f} seconds")

                if self.battery_at_mission_start is not None and self.battery_at_mission_end is not None:
                    used = (self.battery_at_mission_start - self.battery_at_mission_end) * 100.0
                    self.get_logger().info(f"Battery Used: {used:.3f}%")
                else:
                    self.get_logger().warn("Missing start/end battery data!")

            self.state = "DONE"

        elif self.state == "DONE":
            rclpy.shutdown()
            pass

        self.offboard_setpoint_counter += 1

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_cmd_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0
        )
        self.get_logger().info("Offboard mode command sent")


def main(args=None):
    rclpy.init(args=args)
    node = CylinderMission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Interrupted, shutting down.")
    finally:
        if rclpy.ok():  # Ensure it's not already shut down
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
