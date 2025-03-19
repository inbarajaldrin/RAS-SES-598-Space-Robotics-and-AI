#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class FeatureMatcher(Node):
    def __init__(self):
        super().__init__('feature_matcher')
        
        # Create QoS profile for better image streaming
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize ORB detector
        self.orb = cv2.ORB_create(
            nfeatures=1000,
            scaleFactor=1.2,
            nlevels=8,
            edgeThreshold=31,
            firstLevel=0,
            WTA_K=2,
            patchSize=31,
            fastThreshold=20
        )
        
        # Initialize Brute Force Matcher
        self.bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Store latest images
        self.left_image = None
        self.right_image = None
        
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
        
        # Create publisher for visualization
        self.vis_pub = self.create_publisher(
            Image,
            '/dreams_stereo/feature_matches',
            qos_profile
        )
        
        # Create timer for processing
        self.create_timer(0.1, self.process_features)  # 10Hz processing
        
        # Initialize color generator for lines
        self.color_generator = self.generate_colors()
        
        self.get_logger().info('Feature Matcher Node initialized')
    
    def generate_colors(self):
        """Generate random colors for visualization."""
        while True:
            # Generate bright, distinct colors
            hue = np.random.randint(0, 180)
            saturation = np.random.randint(200, 256)
            value = np.random.randint(200, 256)
            # Convert HSV to BGR
            hsv = np.uint8([[[hue, saturation, value]]])
            bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
            yield tuple(map(int, bgr[0][0]))
    
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
    
    def process_features(self):
        if self.left_image is None or self.right_image is None:
            return
            
        try:
            # Detect ORB features and compute descriptors
            kp1, des1 = self.orb.detectAndCompute(self.left_image, None)
            kp2, des2 = self.orb.detectAndCompute(self.right_image, None)
            
            if des1 is None or des2 is None or len(des1) < 2 or len(des2) < 2:
                return
            
            # Match descriptors
            matches = self.bf.match(des1, des2)
            
            # Sort matches by distance
            matches = sorted(matches, key=lambda x: x.distance)
            
            # Keep only good matches (adjust ratio as needed)
            good_matches = matches[:50]  # Keep top 50 matches
            
            # Create visualization
            h1, w1 = self.left_image.shape
            h2, w2 = self.right_image.shape
            vis_img = np.zeros((max(h1, h2), w1 + w2, 3), dtype=np.uint8)
            
            # Convert grayscale to BGR for visualization
            left_bgr = cv2.cvtColor(self.left_image, cv2.COLOR_GRAY2BGR)
            right_bgr = cv2.cvtColor(self.right_image, cv2.COLOR_GRAY2BGR)
            
            # Copy images to visualization
            vis_img[0:h1, 0:w1] = left_bgr
            vis_img[0:h2, w1:w1+w2] = right_bgr
            
            # Draw matches with different colored lines
            for match in good_matches:
                # Get keypoints coordinates
                left_pt = tuple(map(int, kp1[match.queryIdx].pt))
                right_pt = tuple(map(int, kp2[match.trainIdx].pt))
                
                # Adjust right point x-coordinate
                right_pt = (right_pt[0] + w1, right_pt[1])
                
                # Get a color for this line
                color = next(self.color_generator)
                
                # Draw line and circles
                cv2.line(vis_img, left_pt, right_pt, color, 1)
                cv2.circle(vis_img, left_pt, 3, color, -1)
                cv2.circle(vis_img, right_pt, 3, color, -1)
            
            # Convert visualization to ROS message and publish
            vis_msg = self.bridge.cv2_to_imgmsg(vis_img, encoding='bgr8')
            self.vis_pub.publish(vis_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing features: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = FeatureMatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 