#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import control
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import Parameter, ParameterDescriptor, FloatingPointRange
from rclpy.parameter import ParameterType

class CartPoleLQR(Node):
    def __init__(self):
        super().__init__('cart_pole_lqr')
        
        # System parameters
        self.declare_parameter('mass_cart', 1.0)
        self.declare_parameter('mass_pole', 0.1)
        self.declare_parameter('pole_length', 1.0)
        self.declare_parameter('gravity', 9.81)
        
        # LQR weights
        self.declare_parameter('Q_x', 1.0)
        self.declare_parameter('Q_x_dot', 0.1)
        self.declare_parameter('Q_theta', 10.0)
        self.declare_parameter('Q_theta_dot', 0.1)
        self.declare_parameter('R', 1.0)

        # Get initial parameters
        self.update_parameters()
        
        # System state: [x, x_dot, theta, theta_dot]
        # Initialize with pole pointing slightly off vertical
        self.state = np.array([0.0, 0.0, 0.1, 0.0])  # Small initial angle for testing
        
        # Setup publishers for control commands
        self.cart_cmd_pub = self.create_publisher(Float64, '/cart_pole/cart_slider_cmd', 10)
        
        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/cart_pole/joint_states',
            self.joint_state_callback,
            10)
        
        # Create timer for control loop
        self.create_timer(0.02, self.control_loop)  # 50Hz control loop
        
        # Compute initial LQR gains
        self.compute_lqr_gains()
        
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)

    def update_parameters(self):
        self.M = self.get_parameter('mass_cart').value
        self.m = self.get_parameter('mass_pole').value
        self.L = self.get_parameter('pole_length').value
        self.g = self.get_parameter('gravity').value
        
        # Update Q matrix with all configurable weights
        self.Q = np.diag([
            self.get_parameter('Q_x').value,
            self.get_parameter('Q_x_dot').value,
            self.get_parameter('Q_theta').value,
            self.get_parameter('Q_theta_dot').value
        ])
        self.R = np.array([[self.get_parameter('R').value]])

    def compute_lqr_gains(self):
        # Linearized system matrices around the upright equilibrium (theta = 0)
        A = np.array([
            [0, 1, 0, 0],
            [0, 0, self.m * self.g / self.M, 0],  # Positive g term for upright pole
            [0, 0, 0, 1],
            [0, 0, (self.M + self.m) * self.g / (self.M * self.L), 0]  # Positive g term for upright pole
        ])
        
        B = np.array([
            [0],
            [1/self.M],
            [0],
            [-1/(self.M * self.L)]
        ])
        
        # Use Python Control Systems Library for LQR
        K, S, E = control.lqr(A, B, self.Q, self.R)
        self.K = K

    def joint_state_callback(self, msg):
        try:
            # Update state from joint positions and velocities
            cart_idx = msg.name.index('cart_slider')
            pole_idx = msg.name.index('pole_hinge')
            
            self.state[0] = msg.position[cart_idx]  # cart position
            self.state[1] = msg.velocity[cart_idx]  # cart velocity
            self.state[2] = msg.position[pole_idx]  # pole angle
            self.state[3] = msg.velocity[pole_idx]  # pole angular velocity
            
            # Normalize angle to [-pi, pi]
            self.state[2] = ((self.state[2] + np.pi) % (2 * np.pi)) - np.pi
            
        except ValueError as e:
            self.get_logger().warn(f'Joint state callback error: {e}')

    def control_loop(self):
        # Compute control input
        u = -self.K @ self.state
        
        # Publish control command
        cmd_msg = Float64()
        cmd_msg.data = float(u[0])
        self.cart_cmd_pub.publish(cmd_msg)

    def parameter_callback(self, params):
        for param in params:
            if param.name in ['mass_cart', 'mass_pole', 'pole_length', 'gravity', 'Q_x', 'Q_x_dot', 'Q_theta', 'Q_theta_dot', 'R']:
                self.update_parameters()
                self.compute_lqr_gains()
        return True

def main(args=None):
    rclpy.init(args=args)
    node = CartPoleLQR()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 