#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class FeatureTracker(Node):
    """
    A ROS2 node that tracks ORB features in drone camera images.
    """
    def __init__(self):
        super().__init__('feature_tracker')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize ORB detector
        self.orb = cv2.ORB_create(
            nfeatures=500,        # Maximum number of features to detect
            scaleFactor=1.2,      # Scale factor between levels in the scale pyramid
            nlevels=8,            # Number of pyramid levels
            edgeThreshold=31,     # Size of the border where features are not detected
            firstLevel=0,         # Level of pyramid to put source image
            WTA_K=2,             # Number of points that produce each element of the oriented BRIEF descriptor
            scoreType=cv2.ORB_HARRIS_SCORE,  # Algorithm to rank features
            patchSize=31,         # Size of the patch used by the oriented BRIEF descriptor
            fastThreshold=20      # FAST detector threshold
        )

        # Initialize feature matcher
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        
        # Store previous frame data
        self.prev_frame = None
        self.prev_keypoints = None
        self.prev_descriptors = None
        
        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/drone_camera',
            self.image_callback,
            10
        )
        
        # Create publishers for visualization
        self.feature_img_pub = self.create_publisher(
            Image,
            '/feature_tracking/annotated_image',
            10
        )
        
        self.get_logger().info('Feature tracker node initialized')

    def detect_and_compute(self, frame):
        """Detect ORB features and compute descriptors."""
        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect keypoints and compute descriptors
        keypoints, descriptors = self.orb.detectAndCompute(gray, None)
        
        return keypoints, descriptors

    def match_features(self, desc1, desc2):
        """Match features between two frames."""
        if desc1 is None or desc2 is None:
            return []
            
        # Match descriptors
        matches = self.matcher.match(desc1, desc2)
        
        # Sort matches by distance
        matches = sorted(matches, key=lambda x: x.distance)
        
        return matches

    def draw_features(self, frame, keypoints, matches=None, prev_keypoints=None):
        """Draw detected features and matches on the frame."""
        # Create output image
        output = frame.copy()
        
        if matches is not None and prev_keypoints is not None:
            # Draw matches
            for match in matches[:30]:  # Draw top 30 matches
                prev_pt = prev_keypoints[match.trainIdx].pt
                curr_pt = keypoints[match.queryIdx].pt
                
                # Draw line between matched features
                cv2.line(output, 
                        (int(prev_pt[0]), int(prev_pt[1])),
                        (int(curr_pt[0]), int(curr_pt[1])),
                        (0, 255, 0), 2)
                
                # Draw circles at feature points
                cv2.circle(output, (int(prev_pt[0]), int(prev_pt[1])), 4, (255, 0, 0), -1)
                cv2.circle(output, (int(curr_pt[0]), int(curr_pt[1])), 4, (0, 0, 255), -1)
        else:
            # Just draw current keypoints
            cv2.drawKeypoints(output, keypoints, output, 
                            color=(0, 255, 0),
                            flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        return output

    def image_callback(self, msg):
        """Process incoming image messages."""
        try:
            # Convert ROS Image message to OpenCV image
            current_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect features in current frame
            curr_keypoints, curr_descriptors = self.detect_and_compute(current_frame)
            
            # Match features with previous frame if available
            matches = None
            if self.prev_descriptors is not None:
                matches = self.match_features(curr_descriptors, self.prev_descriptors)
            
            # Draw features and matches
            annotated_frame = self.draw_features(current_frame, curr_keypoints, 
                                               matches, self.prev_keypoints)
            
            # Publish annotated image
            self.feature_img_pub.publish(
                self.bridge.cv2_to_imgmsg(annotated_frame, "bgr8")
            )
            
            # Update previous frame data
            self.prev_frame = current_frame
            self.prev_keypoints = curr_keypoints
            self.prev_descriptors = curr_descriptors
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main():
    rclpy.init()
    
    feature_tracker = FeatureTracker()
    
    try:
        rclpy.spin(feature_tracker)
    except KeyboardInterrupt:
        pass
    finally:
        feature_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 