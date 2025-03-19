#!/usr/bin/env python3

import cv2
import numpy as np
import os

def generate_aruco_marker():
    """Generate ArUco marker and save it as a PNG file."""
    # Create ArUco dictionary and parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    
    # Generate marker image (ID 0)
    marker_size = 400  # pixels
    marker_image = np.zeros((marker_size, marker_size), dtype=np.uint8)
    marker_image = cv2.aruco.generateImageMarker(aruco_dict, 0, marker_size, marker_image, 1)
    
    # Add white border for better visibility
    border_size = 50
    final_size = marker_size + 2 * border_size
    final_image = np.ones((final_size, final_size), dtype=np.uint8) * 255
    final_image[border_size:border_size+marker_size, border_size:border_size+marker_size] = marker_image
    
    # Create materials directory if it doesn't exist
    materials_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'models', 'cylinder', 'materials'
    )
    os.makedirs(materials_dir, exist_ok=True)
    
    # Save the marker
    output_path = os.path.join(materials_dir, 'aruco_marker_0.png')
    cv2.imwrite(output_path, final_image)
    print(f"Generated ArUco marker at: {output_path}")

if __name__ == '__main__':
    generate_aruco_marker() 