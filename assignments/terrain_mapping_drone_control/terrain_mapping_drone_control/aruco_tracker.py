#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import TransformStamped, Point
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String
from transforms3d.euler import mat2euler
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ArucoTracker(Node):
    def __init__(self):
        super().__init__('aruco_tracker')
        
        # Initialize CV bridge
        self.cv_bridge = CvBridge()
        
        # Frame counter for logging
        self.frame_counter = 0
        
        # Print OpenCV version for debugging
        self.get_logger().info(f'OpenCV version: {cv2.__version__}')
        
        try:
            # ArUco dictionary and parameters (OpenCV 4.7+)
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            self.get_logger().info('Using OpenCV 4.7+ ArUco API with 4x4 dictionary')
        except AttributeError:
            # Fallback for older OpenCV versions
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.detector = None
            self.get_logger().info('Using older OpenCV ArUco API with 4x4 dictionary')
        
        # Camera matrix and distortion coefficients (default values)
        # Default calibration for a typical 640x480 camera
        self.camera_matrix = np.array([
            [554.254691191187, 0.0, 320.5],
            [0.0, 554.254691191187, 240.5],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.zeros(5)  # Default to no distortion
        self.calibration_received = False
        
        self.marker_size = 0.8  # 80cm marker size as per Gazebo model
        
        # Publishers
        self.debug_image_pub = self.create_publisher(Image, '/aruco/debug_image', 10)
        self.marker_pose_pub = self.create_publisher(String, '/aruco/marker_pose', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.create_subscription(
            Image,
            '/drone/down_mono',
            self.image_callback,
            10
        )
        
        # Create camera info subscriber with QoS profile for reliable delivery
        camera_info_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(
            CameraInfo,
            '/drone/down_mono/camera_info',
            self.camera_info_callback,
            qos_profile=camera_info_qos
        )
        
        self.get_logger().info('ArUco tracker node initialized with default calibration')
        self.get_logger().info(f'Default camera matrix:\n{self.camera_matrix}')

    def detect_markers(self, image):
        """Detect ArUco markers using the appropriate OpenCV API."""
        try:
            if self.detector is not None:
                # OpenCV 4.7+ API
                corners, ids, rejected = self.detector.detectMarkers(image)
            else:
                # Older OpenCV API
                corners, ids, rejected = cv2.aruco.detectMarkers(
                    image, self.aruco_dict, parameters=self.aruco_params)
            
            if ids is not None:
                self.get_logger().info(f'Detected {len(ids)} markers')
            return corners, ids, rejected
        except Exception as e:
            self.get_logger().error(f'Error in marker detection: {str(e)}')
            return [], None, []

    def draw_crosshair(self, image, center, size=20, color=(0,0,255), thickness=2):
        """Draw a crosshair at the specified center point."""
        x, y = int(center[0]), int(center[1])
        # Draw horizontal line
        cv2.line(image, (x - size, y), (x + size, y), color, thickness)
        # Draw vertical line
        cv2.line(image, (x, y - size), (x, y + size), color, thickness)
        # Draw small circle at center
        cv2.circle(image, (x, y), 2, color, thickness)

    def image_callback(self, msg):
        """Process incoming image and detect ArUco markers."""
        if not self.calibration_received:
            self.get_logger().debug('Using default camera calibration')
            
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'mono8')
            
            # Log image properties
            if self.frame_counter % 100 == 0:  # Log every 100 frames
                self.get_logger().debug(f'Image shape: {cv_image.shape}, dtype: {cv_image.dtype}')
            
            # Increment frame counter
            self.frame_counter += 1
            
            # Detect ArUco markers
            corners, ids, rejected = self.detect_markers(cv_image)
            
            # Create debug image
            debug_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            
            if ids is not None and len(ids) > 0:
                # Draw detected markers
                if self.detector is not None:
                    cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)
                else:
                    cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)
                
                try:
                    # Estimate pose for each marker
                    if self.detector is not None:
                        # OpenCV 4.7+ API
                        for i in range(len(ids)):
                            marker_points = np.array([
                                [-self.marker_size/2, self.marker_size/2, 0],
                                [self.marker_size/2, self.marker_size/2, 0],
                                [self.marker_size/2, -self.marker_size/2, 0],
                                [-self.marker_size/2, -self.marker_size/2, 0]], dtype=np.float32)
                            objPoints = marker_points.reshape((4,3))
                            imgPoints = corners[i].reshape((4,2))
                            
                            # Get pose for single marker
                            success, rvec, tvec = cv2.solvePnP(
                                objPoints, imgPoints, self.camera_matrix, self.dist_coeffs)
                            
                            if success:
                                # Draw axis for each marker
                                cv2.drawFrameAxes(debug_image, self.camera_matrix, self.dist_coeffs, 
                                                rvec, tvec, self.marker_size/2)
                                
                                # Get marker position in camera frame
                                marker_position = tvec.flatten()
                                
                                # Convert rotation vector to rotation matrix
                                rot_matrix, _ = cv2.Rodrigues(rvec)
                                
                                # Get Euler angles from rotation matrix
                                euler_angles = mat2euler(rot_matrix)
                                
                                # Convert to quaternion
                                quat = self.euler_to_quaternion(euler_angles[0], euler_angles[1], euler_angles[2])
                                
                                # Publish transform
                                transform = TransformStamped()
                                transform.header.stamp = self.get_clock().now().to_msg()
                                transform.header.frame_id = 'camera_frame'
                                transform.child_frame_id = f'aruco_marker_{ids[i][0]}'
                                
                                transform.transform.translation.x = marker_position[0]
                                transform.transform.translation.y = marker_position[1]
                                transform.transform.translation.z = marker_position[2]
                                
                                transform.transform.rotation.x = quat[0]
                                transform.transform.rotation.y = quat[1]
                                transform.transform.rotation.z = quat[2]
                                transform.transform.rotation.w = quat[3]
                                
                                self.tf_broadcaster.sendTransform(transform)
                                
                                # Publish marker pose as text
                                pose_msg = String()
                                pose_msg.data = (
                                    f"Marker {ids[i][0]} detected at "
                                    f"x:{marker_position[0]:.2f}m, "
                                    f"y:{marker_position[1]:.2f}m, "
                                    f"z:{marker_position[2]:.2f}m"
                                )
                                self.marker_pose_pub.publish(pose_msg)
                                
                                # Calculate and draw marker center
                                marker_center = corners[i][0].mean(axis=0)
                                self.draw_crosshair(debug_image, marker_center, size=20, color=(0,0,255), thickness=2)
                                
                                # Draw position text above crosshair
                                text_position = (int(marker_center[0]), int(marker_center[1] - 30))
                                cv2.putText(debug_image,
                                    f"id:{ids[i][0]} x:{marker_position[0]:.2f} y:{marker_position[1]:.2f} z:{marker_position[2]:.2f}",
                                    text_position,
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.5,
                                    (0, 255, 0),
                                    2
                                )
                    else:
                        # Older OpenCV API
                        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                            corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
                        
                        for i in range(len(ids)):
                            # Draw axis for each marker
                            cv2.drawFrameAxes(debug_image, self.camera_matrix, self.dist_coeffs, 
                                            rvecs[i], tvecs[i], self.marker_size/2)
                            
                            # Get marker position in camera frame
                            marker_position = tvecs[i][0]
                            
                            # Convert rotation vector to rotation matrix
                            rot_matrix, _ = cv2.Rodrigues(rvecs[i][0])
                            
                            # Get Euler angles from rotation matrix
                            euler_angles = mat2euler(rot_matrix)
                            
                            # Convert to quaternion
                            quat = self.euler_to_quaternion(euler_angles[0], euler_angles[1], euler_angles[2])
                            
                            # Publish transform
                            transform = TransformStamped()
                            transform.header.stamp = self.get_clock().now().to_msg()
                            transform.header.frame_id = 'camera_frame'
                            transform.child_frame_id = f'aruco_marker_{ids[i][0]}'
                            
                            transform.transform.translation.x = marker_position[0]
                            transform.transform.translation.y = marker_position[1]
                            transform.transform.translation.z = marker_position[2]
                            
                            transform.transform.rotation.x = quat[0]
                            transform.transform.rotation.y = quat[1]
                            transform.transform.rotation.z = quat[2]
                            transform.transform.rotation.w = quat[3]
                            
                            self.tf_broadcaster.sendTransform(transform)
                            
                            # Publish marker pose as text
                            pose_msg = String()
                            pose_msg.data = (
                                f"Marker {ids[i][0]} detected at "
                                f"x:{marker_position[0]:.2f}m, "
                                f"y:{marker_position[1]:.2f}m, "
                                f"z:{marker_position[2]:.2f}m"
                            )
                            self.marker_pose_pub.publish(pose_msg)
                            
                            # Calculate and draw marker center
                            marker_center = corners[i][0].mean(axis=0)
                            self.draw_crosshair(debug_image, marker_center, size=20, color=(0,0,255), thickness=2)
                            
                            # Draw position text above crosshair
                            text_position = (int(marker_center[0]), int(marker_center[1] - 30))
                            cv2.putText(debug_image,
                                f"id:{ids[i][0]} x:{marker_position[0]:.2f} y:{marker_position[1]:.2f} z:{marker_position[2]:.2f}",
                                text_position,
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 255, 0),
                                2
                            )
                except Exception as e:
                    self.get_logger().error(f'Error in pose estimation: {str(e)}')
            
            # Publish debug image
            debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [x, y, z, w]

    def camera_info_callback(self, msg):
        """Process camera calibration data."""
        try:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.calibration_received = True
            self.get_logger().info('Camera calibration received')
            self.get_logger().info(f'Updated camera matrix:\n{self.camera_matrix}')
            self.get_logger().info(f'Distortion coefficients: {self.dist_coeffs}')
        except Exception as e:
            self.get_logger().error(f'Error processing camera calibration: {str(e)}')

def main():
    rclpy.init()
    node = ArucoTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 