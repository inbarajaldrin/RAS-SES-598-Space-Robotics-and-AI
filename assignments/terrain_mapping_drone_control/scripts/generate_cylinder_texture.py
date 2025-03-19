#!/usr/bin/env python3

import cv2
import numpy as np
import os

def create_feature_rich_texture(width=1024, height=2048):
    """
    Create a texture with good features for ORB detection and optical flow.
    The texture will wrap around the cylinder (width) and cover its height.
    """
    # Create base image
    texture = np.ones((height, width), dtype=np.uint8) * 200

    # Add random noise base
    noise = np.random.randint(0, 50, (height, width), dtype=np.uint8)
    texture = cv2.add(texture, noise)

    # Add gradient patterns
    for i in range(0, height, 100):
        cv2.line(texture, (0, i), (width, i+50), (100,), 2)

    # Add circles of varying sizes
    for _ in range(50):
        x = np.random.randint(0, width)
        y = np.random.randint(0, height)
        radius = np.random.randint(5, 30)
        cv2.circle(texture, (x, y), radius, (50,), -1)

    # Add random rectangles
    for _ in range(30):
        x = np.random.randint(0, width-50)
        y = np.random.randint(0, height-50)
        w = np.random.randint(20, 100)
        h = np.random.randint(20, 100)
        cv2.rectangle(texture, (x, y), (x+w, y+h), (150,), -1)

    # Add some text markers at different scales
    fonts = [cv2.FONT_HERSHEY_SIMPLEX, cv2.FONT_HERSHEY_COMPLEX]
    for _ in range(40):
        text = "+"
        x = np.random.randint(0, width-20)
        y = np.random.randint(0, height-20)
        scale = np.random.uniform(0.5, 2.0)
        font = fonts[np.random.randint(0, len(fonts))]
        cv2.putText(texture, text, (x, y), font, scale, (0,), 2)

    # Add some diagonal lines
    for _ in range(40):
        x1 = np.random.randint(0, width)
        y1 = np.random.randint(0, height)
        x2 = x1 + np.random.randint(-100, 100)
        y2 = y1 + np.random.randint(-100, 100)
        cv2.line(texture, (x1, y1), (x2, y2), (100,), 2)

    # Ensure the texture wraps horizontally (for cylinder mapping)
    blend_width = 50
    left_edge = texture[:, :blend_width].copy()
    right_edge = texture[:, -blend_width:].copy()
    for i in range(blend_width):
        alpha = i / blend_width
        texture[:, i] = right_edge[:, i] * (1-alpha) + left_edge[:, i] * alpha
        texture[:, -(i+1)] = left_edge[:, -(i+1)] * (1-alpha) + right_edge[:, -(i+1)] * alpha

    # Apply Gaussian blur to smooth out harsh edges
    texture = cv2.GaussianBlur(texture, (3, 3), 0)

    # Create color version
    texture_color = cv2.cvtColor(texture, cv2.COLOR_GRAY2BGR)
    
    # Add subtle color variations for visual interest while maintaining feature detectability
    color_overlay = np.random.randint(0, 30, texture_color.shape, dtype=np.uint8)
    texture_color = cv2.add(texture_color, color_overlay)

    # Save the texture
    materials_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'models', 'cylinder', 'materials'
    )
    os.makedirs(materials_dir, exist_ok=True)
    
    output_path = os.path.join(materials_dir, 'cylinder_texture.png')
    cv2.imwrite(output_path, texture_color)
    print(f"Generated cylinder texture at: {output_path}")

    # Test feature detection
    orb = cv2.ORB_create()
    keypoints = orb.detect(texture, None)
    print(f"Detected {len(keypoints)} ORB features in the texture")

if __name__ == '__main__':
    create_feature_rich_texture() 