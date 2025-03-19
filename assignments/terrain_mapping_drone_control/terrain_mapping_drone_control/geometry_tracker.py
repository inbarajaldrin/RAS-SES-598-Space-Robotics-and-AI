#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point

class GeometryTracker(Node):
    def __init__(self):
        super().__init__('geometry_tracker')
        
        # Initialize CV bridge
        self.cv_bridge = CvBridge()
        
        # Subscribers
        self.depth_image_subscriber = self.create_subscription(
            Image, '/drone/front_depth',
            self.depth_image_callback, 10)
            
        # Publishers
        self.debug_image_pub = self.create_publisher(Image, '/geometry/debug_image', 10)
        self.cylinder_pose_pub = self.create_publisher(Point, '/geometry/cylinder_center', 10)
        self.cylinder_info_pub = self.create_publisher(Float32MultiArray, '/geometry/cylinder_info', 10)
        
        self.get_logger().info('Geometry tracker node initialized')

    def depth_image_callback(self, msg):
        """Process depth image to detect geometric shapes."""
        try:
            # Convert ROS Image to OpenCV format - for depth data (32FC1)
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            
            # Create a copy for visualization and processing
            depth_display = depth_image.copy()
            
            # Replace NaN values with 0 for visualization
            depth_display[np.isnan(depth_display)] = 0
            
            # Set fixed depth range for better visualization (0-10 meters is a good range for indoor scenes)
            depth_min = 0.0
            depth_max = 10.0  # Maximum depth range to visualize
            
            # Clip depth values to the range and normalize
            depth_normalized = np.clip(depth_display, depth_min, depth_max)
            depth_normalized = ((depth_normalized - depth_min) * 255 / (depth_max - depth_min))
            depth_normalized = depth_normalized.astype(np.uint8)
            
            # Apply histogram equalization to enhance contrast
            depth_normalized = cv2.equalizeHist(depth_normalized)
            
            # Create debug image (BGR format for visualization)
            debug_image = cv2.cvtColor(depth_normalized, cv2.COLOR_GRAY2BGR)
            
            # Apply Gaussian blur to reduce noise
            blurred = cv2.GaussianBlur(depth_normalized, (5, 5), 0)
            
            # Edge detection using Canny
            edges = cv2.Canny(blurred, 50, 150)
            
            # Find vertical lines using Hough transform
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=100, maxLineGap=10)
            vertical_lines = []
            
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    # Calculate line angle
                    angle = np.abs(np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi)
                    
                    # Filter for near-vertical lines (within 20 degrees of vertical)
                    if angle > 70 and angle < 110:
                        vertical_lines.append(line[0])
                        # Draw vertical lines in red
                        cv2.line(debug_image, (x1,y1), (x2,y2), (0,0,255), 2)
                        # Add line angle text
                        cv2.putText(debug_image, 
                                  f'{angle:.1f}°', 
                                  (x1, y1-10), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 
                                  0.5, 
                                  (0,0,255), 
                                  1)
            
            # Find contours for cylinder detection
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Draw all contours in green (light)
            cv2.drawContours(debug_image, contours, -1, (0,100,0), 1)
            
            # Process contours to find cylinder-like shapes
            best_cylinder = None
            best_confidence = 0.0
            
            for contour in contours:
                # Calculate contour properties
                area = cv2.contourArea(contour)
                
                # Filter small contours
                if area < 1000:  # Minimum area threshold
                    continue
                
                # Fit an ellipse to the contour if possible
                if len(contour) >= 5:
                    ellipse = cv2.fitEllipse(contour)
                    (center_x, center_y), (width, height), angle = ellipse
                    
                    # Calculate aspect ratio and confidence
                    aspect_ratio = min(width, height) / max(width, height)
                    angle_confidence = 1.0 - abs(angle - 90) / 90
                    size_confidence = min(width, height) / 100  # Normalize by expected size
                    confidence = aspect_ratio * angle_confidence * size_confidence
                    
                    # Filter for cylinder-like shapes (nearly vertical ellipses)
                    if aspect_ratio > 0.3 and abs(angle - 90) < 30:
                        # Update best cylinder if confidence is higher
                        if confidence > best_confidence:
                            best_cylinder = (ellipse, confidence)
                            best_confidence = confidence
                        
                        # Draw the ellipse in blue
                        cv2.ellipse(debug_image, ellipse, (255,0,0), 2)
                        
                        # Get depth at center of ellipse
                        center_x, center_y = int(center_x), int(center_y)
                        if 0 <= center_y < depth_image.shape[0] and 0 <= center_x < depth_image.shape[1]:
                            center_depth = depth_image[center_y, center_x]
                            if not np.isnan(center_depth):  # Only show valid depth values
                                # Draw crosshair at center
                                cv2.line(debug_image, (center_x-15, center_y), (center_x+15, center_y), (255,255,0), 2)
                                cv2.line(debug_image, (center_x, center_y-15), (center_x, center_y+15), (255,255,0), 2)
                                
                                # Add text for measurements
                                cv2.putText(debug_image, 
                                          f'D:{center_depth:.2f}m AR:{aspect_ratio:.2f} C:{confidence:.2f}', 
                                          (center_x-50, center_y-20), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 
                                          0.5, 
                                          (255,255,0), 
                                          1)
            
            # Publish best cylinder info if found
            if best_cylinder is not None:
                ellipse, confidence = best_cylinder
                (center_x, center_y), (width, height), angle = ellipse
                center_x, center_y = int(center_x), int(center_y)
                
                if 0 <= center_y < depth_image.shape[0] and 0 <= center_x < depth_image.shape[1]:
                    center_depth = depth_image[center_y, center_x]
                    if not np.isnan(center_depth):
                        # Publish cylinder center
                        center_msg = Point()
                        center_msg.x = float(center_x)
                        center_msg.y = float(center_y)
                        center_msg.z = float(center_depth)
                        self.cylinder_pose_pub.publish(center_msg)
                        
                        # Publish cylinder info (width, height, angle, confidence)
                        info_msg = Float32MultiArray()
                        info_msg.data = [float(width), float(height), float(angle), float(confidence)]
                        self.cylinder_info_pub.publish(info_msg)
            
            # Draw center crosshair
            height, width = depth_image.shape[:2]
            center_y, center_x = height // 2, width // 2
            cv2.line(debug_image, (center_x-20, center_y), (center_x+20, center_y), (0,255,255), 2)
            cv2.line(debug_image, (center_x, center_y-20), (center_x, center_y+20), (0,255,255), 2)
            
            # Add center depth text
            center_depth = depth_image[center_y, center_x]
            if not np.isnan(center_depth):
                cv2.putText(debug_image, 
                          f'Center: {center_depth:.2f}m', 
                          (10, height-10), 
                          cv2.FONT_HERSHEY_SIMPLEX, 
                          1.0, 
                          (0,255,255), 
                          2)
            
            # Add legend
            cv2.putText(debug_image, "Red: Vertical Lines", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
            cv2.putText(debug_image, "Blue: Cylinder Fits", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 1)
            cv2.putText(debug_image, "Yellow: Centers", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
            
            # Add depth range info
            cv2.putText(debug_image, 
                      f'Depth range: {depth_min:.2f}m - {depth_max:.2f}m', 
                      (10, 80), 
                      cv2.FONT_HERSHEY_SIMPLEX, 
                      0.5, 
                      (255,255,255), 
                      1)
            
            # Publish debug image
            debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

def main():
    rclpy.init()
    node = GeometryTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 