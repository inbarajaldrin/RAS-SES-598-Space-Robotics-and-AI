#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class StereoProcessor(Node):
    def __init__(self):
        super().__init__('stereo_processor')
        
        # Create QoS profile for better image streaming
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create subscribers for stereo images
        self.left_sub = self.create_subscription(
            Image,
            '/zed/zed_node/left_gray/image_rect_gray',
            self.left_image_callback,
            qos_profile
        )
        self.right_sub = self.create_subscription(
            Image,
            '/zed/zed_node/right_gray/image_rect_gray',
            self.right_image_callback,
            qos_profile
        )
        
        # Create publishers for disparity and depth
        self.disparity_pub = self.create_publisher(
            Image,
            '/dreams_stereo/disparity',
            qos_profile
        )
        self.depth_pub = self.create_publisher(
            Image,
            '/dreams_stereo/depth',
            qos_profile
        )
        
        # Initialize stereo matcher
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=128,
            blockSize=5,
            P1=8 * 3 * 5**2,
            P2=32 * 3 * 5**2,
            disp12MaxDiff=1,
            uniquenessRatio=15,
            speckleWindowSize=100,
            speckleRange=2
        )
        
        # Store latest images
        self.left_image = None
        self.right_image = None
        
        # Create timer for processing
        self.create_timer(0.1, self.process_stereo)  # 10Hz processing
        
        self.get_logger().info('Stereo Processor Node initialized')
    
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
    
    def process_stereo(self):
        if self.left_image is None or self.right_image is None:
            return
            
        try:
            # Compute disparity
            disparity = self.stereo.compute(self.left_image, self.right_image)
            
            # Normalize disparity for visualization
            disparity_norm = cv2.normalize(disparity, None, alpha=0, beta=255,
                                         norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            # Convert disparity to depth (simplified, needs camera parameters for accurate depth)
            # depth = (baseline * focal_length) / disparity
            # This is a placeholder calculation
            depth = np.zeros_like(disparity, dtype=np.float32)
            mask = disparity > 0
            depth[mask] = 1.0 / (disparity[mask] + 1e-6)  # Simplified depth calculation
            
            # Normalize depth for visualization
            depth_norm = cv2.normalize(depth, None, alpha=0, beta=255,
                                     norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            # Publish results
            disparity_msg = self.bridge.cv2_to_imgmsg(disparity_norm, encoding='mono8')
            depth_msg = self.bridge.cv2_to_imgmsg(depth_norm, encoding='mono8')
            
            self.disparity_pub.publish(disparity_msg)
            self.depth_pub.publish(depth_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing stereo: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = StereoProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 