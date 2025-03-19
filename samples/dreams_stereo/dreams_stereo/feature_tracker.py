#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Transform, Vector3, Quaternion
import tf2_ros
from scipy.spatial.transform import Rotation

class FeatureTracker(Node):
    def __init__(self):
        super().__init__('feature_tracker')
        
        # Create QoS profile for better image streaming
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Initialize CV bridge and ORB detector
        self.bridge = CvBridge()
        self.orb = cv2.ORB_create(nfeatures=1000)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Store latest images and camera info
        self.left_image = None
        self.right_image = None
        self.left_camera_info = None
        self.right_camera_info = None
        
        # Create subscribers
        self.left_sub = self.create_subscription(
            Image, '/zed/zed_node/left_gray/image_rect_gray',
            self.left_image_callback, qos_profile)
        self.right_sub = self.create_subscription(
            Image, '/zed/zed_node/right_gray/image_rect_gray',
            self.right_image_callback, qos_profile)
        self.left_info_sub = self.create_subscription(
            CameraInfo, '/zed/zed_node/left/camera_info',
            self.left_camera_info_callback, qos_profile)
        self.right_info_sub = self.create_subscription(
            CameraInfo, '/zed/zed_node/right/camera_info',
            self.right_camera_info_callback, qos_profile)
        
        # Create publisher for visualization
        self.vis_pub = self.create_publisher(
            Image, '/dreams_stereo/extrinsics_comparison', qos_profile)
        
        # TF buffer for getting published extrinsics
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.create_timer(0.1, self.process_features)
        self.get_logger().info('Feature Tracker Node initialized')

    def left_image_callback(self, msg):
        try:
            self.left_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'Error converting left image: {str(e)}')
    
    def right_image_callback(self, msg):
        try:
            self.right_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f'Error converting right image: {str(e)}')
    
    def left_camera_info_callback(self, msg):
        self.left_camera_info = msg
    
    def right_camera_info_callback(self, msg):
        self.right_camera_info = msg
    
    def normalize_points(self, points):
        """Normalize points for better numerical stability."""
        centroid = np.mean(points, axis=0)
        scale = np.sqrt(2) / np.mean(np.linalg.norm(points - centroid, axis=1))
        T = np.array([[scale, 0, -scale*centroid[0]],
                     [0, scale, -scale*centroid[1]],
                     [0, 0, 1]])
        normalized_points = (points - centroid) * scale
        return normalized_points, T

    def estimate_E_ransac(self, pts1, pts2, K1, K2, num_iterations=1000):
        """Estimate Essential matrix using RANSAC."""
        best_E = None
        best_inliers = []
        best_score = 0
        
        # Normalize points
        pts1_norm = np.linalg.inv(K1) @ np.hstack((pts1, np.ones((pts1.shape[0], 1)))).T
        pts2_norm = np.linalg.inv(K2) @ np.hstack((pts2, np.ones((pts2.shape[0], 1)))).T
        pts1_norm = pts1_norm.T[:, :2] / pts1_norm.T[:, 2:]
        pts2_norm = pts2_norm.T[:, :2] / pts2_norm.T[:, 2:]
        
        for _ in range(num_iterations):
            # Randomly select 8 points
            idx = np.random.choice(pts1.shape[0], 8, replace=False)
            sample1, sample2 = pts1_norm[idx], pts2_norm[idx]
            
            # Compute E using 8-point algorithm
            A = np.zeros((8, 9))
            for i in range(8):
                x1, y1 = sample1[i]
                x2, y2 = sample2[i]
                A[i] = [x2*x1, x2*y1, x2, y2*x1, y2*y1, y2, x1, y1, 1]
            
            _, _, Vt = np.linalg.svd(A)
            E = Vt[-1].reshape(3, 3)
            
            # Enforce Essential matrix properties
            U, S, Vt = np.linalg.svd(E)
            S = np.diag([1, 1, 0])
            E = U @ S @ Vt
            
            # Count inliers using Sampson distance
            x1 = np.hstack((pts1_norm, np.ones((pts1_norm.shape[0], 1))))
            x2 = np.hstack((pts2_norm, np.ones((pts2_norm.shape[0], 1))))
            
            Ex1 = x1 @ E.T
            Ex2 = x2 @ E
            Exx = np.sum(x2 * Ex1, axis=1)
            sampson_dist = Exx**2 / (np.sum(Ex1[:, :2]**2, axis=1) + np.sum(Ex2[:, :2]**2, axis=1))
            
            inliers = sampson_dist < 0.01
            score = np.sum(inliers)
            
            if score > best_score:
                best_score = score
                best_E = E
                best_inliers = inliers
        
        return best_E, best_inliers

    def decompose_E(self, E):
        """Decompose Essential matrix into R and t."""
        U, _, Vt = np.linalg.svd(E)
        W = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        
        # Get possible R and t
        R1 = U @ W @ Vt
        R2 = U @ W.T @ Vt
        t = U[:, 2]
        
        # Ensure proper rotation matrix
        if np.linalg.det(R1) < 0: R1 = -R1
        if np.linalg.det(R2) < 0: R2 = -R2
        
        return [R1, R2], [t, -t]

    def draw_comparison(self, img1, img2, kp1, kp2, matches, R_est, t_est, R_pub, t_pub):
        """Draw feature matches and extrinsics comparison."""
        h, w = img1.shape[:2]
        vis_img = np.zeros((h, w*2 + 200, 3), dtype=np.uint8)
        
        # Draw images side by side
        vis_img[:, :w] = cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)
        vis_img[:, w:2*w] = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)
        
        # Draw matches
        for match in matches:
            pt1 = tuple(map(int, kp1[match.queryIdx].pt))
            pt2 = tuple(map(int, [kp2[match.trainIdx].pt[0] + w, kp2[match.trainIdx].pt[1]]))
            cv2.line(vis_img, pt1, pt2, (0, 255, 0), 1)
        
        # Draw extrinsics comparison with smaller font
        text_x = int(2*w + 5)  # Convert to int
        font_scale = 0.4
        line_height = 15
        
        y = int(20)  # Convert to int
        cv2.putText(vis_img, "Est:", (text_x, y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0,255,0), 1)
        y = int(y + line_height)  # Convert to int
        euler = np.rad2deg(Rotation.from_matrix(R_est).as_euler('xyz'))
        cv2.putText(vis_img, f"R:[{euler[0]:.1f},{euler[1]:.1f},{euler[2]:.1f}]", 
                   (text_x, y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0,255,0), 1)
        y = int(y + line_height)  # Convert to int
        cv2.putText(vis_img, f"t:[{t_est[0]:.3f},{t_est[1]:.3f},{t_est[2]:.3f}]", 
                   (text_x, y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0,255,0), 1)
        
        y = int(y + line_height * 1.5)  # Convert to int
        cv2.putText(vis_img, "Pub:", (text_x, y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0,255,255), 1)
        y = int(y + line_height)  # Convert to int
        euler = np.rad2deg(Rotation.from_matrix(R_pub).as_euler('xyz'))
        cv2.putText(vis_img, f"R:[{euler[0]:.1f},{euler[1]:.1f},{euler[2]:.1f}]", 
                   (text_x, y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0,255,255), 1)
        y = int(y + line_height)  # Convert to int
        cv2.putText(vis_img, f"t:[{t_pub[0]:.3f},{t_pub[1]:.3f},{t_pub[2]:.3f}]", 
                   (text_x, y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0,255,255), 1)
        
        return vis_img

    def process_features(self):
        # Check if images are available
        if not all([
            isinstance(self.left_image, np.ndarray),
            isinstance(self.right_image, np.ndarray),
            self.left_camera_info is not None,
            self.right_camera_info is not None
        ]):
            return
            
        try:
            # Detect and match ORB features
            kp1, des1 = self.orb.detectAndCompute(self.left_image, None)
            kp2, des2 = self.orb.detectAndCompute(self.right_image, None)
            
            if des1 is None or des2 is None or len(kp1) < 8 or len(kp2) < 8:
                return
                
            matches = self.matcher.match(des1, des2)
            matches = sorted(matches, key=lambda x: x.distance)[:100]  # Keep best matches
            
            # Get matched points
            pts1 = np.float32([kp1[m.queryIdx].pt for m in matches])
            pts2 = np.float32([kp2[m.trainIdx].pt for m in matches])
            
            # Get camera matrices
            K1 = np.array(self.left_camera_info.k).reshape(3, 3)
            K2 = np.array(self.right_camera_info.k).reshape(3, 3)
            
            # Estimate Essential matrix using RANSAC
            E, inliers = self.estimate_E_ransac(pts1, pts2, K1, K2)
            
            # Decompose E to get R and t
            Rs, ts = self.decompose_E(E)
            
            # Get published transform
            try:
                transform = self.tf_buffer.lookup_transform(
                    'zed_left_camera_frame',
                    'zed_right_camera_frame',
                    rclpy.time.Time())
                
                # Convert to R and t
                q = transform.transform.rotation
                t_pub = np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ])
                R_pub = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                
                # Choose correct R and t from decomposition (closest to published)
                best_R = Rs[0]
                best_t = ts[0]
                min_diff = float('inf')
                
                for R, t in zip(Rs, ts):
                    diff = np.linalg.norm(R - R_pub) + np.linalg.norm(t - t_pub)
                    if diff < min_diff:
                        min_diff = diff
                        best_R = R
                        best_t = t
                
                # Draw visualization
                vis_img = self.draw_comparison(
                    self.left_image, self.right_image,
                    kp1, kp2, matches, best_R, best_t, R_pub, t_pub
                )
                
                # Publish visualization
                vis_msg = self.bridge.cv2_to_imgmsg(vis_img, encoding='bgr8')
                self.vis_pub.publish(vis_msg)
                
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                self.get_logger().error(f'TF error: {str(e)}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing features: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = FeatureTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 