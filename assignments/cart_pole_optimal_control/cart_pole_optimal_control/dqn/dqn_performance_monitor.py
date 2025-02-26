#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
import matplotlib.pyplot as plt
from collections import deque

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')

        # Initialize subscribers
        self.create_subscription(JointState, '/dqn/cart_pole/joint_state', self.joint_state_callback, 10)
        self.create_subscription(Float64, '/dqn/cart_pole/cmd_force', self.control_callback, 10)

        self.create_subscription(Float64, '/dqn/earthquake_force', self.earthquake_callback, 10)

        # Performance Metrics Storage
        self.angle_deviation = deque(maxlen=1000)  # Stores recent pole angle deviations
        self.cart_position = deque(maxlen=1000)   # Stores recent cart positions
        self.control_effort = deque(maxlen=1000)  # Stores recent control force values
        self.earthquake_force = deque(maxlen=1000) # Stores recent earthquake forces

        self.simulation_active = True  # Track if the simulation is still valid
        self.max_cart_position = 2.5  # Define cart position limit

        # Initialize Matplotlib figure
        self.fig, self.axs = plt.subplots(2, 2, figsize=(10, 6))
        self.timer = self.create_timer(0.1, self.update_plot)  # Update plot every 0.1s

        self.get_logger().info('Performance Monitor Node Started')

    def joint_state_callback(self, msg):
        """Processes joint state updates and tracks performance."""
        if not self.simulation_active:
            return  # Stop updating if simulation has exceeded bounds

        try:
            cart_idx = msg.name.index('cart_to_base')
            pole_idx = msg.name.index('pole_joint')

            cart_x = msg.position[cart_idx]
            theta = np.rad2deg(msg.position[pole_idx])  # Convert radians to degrees

            # **Check if the cart has exceeded limits**
            if abs(cart_x) >= self.max_cart_position:
                self.simulation_active = False
                self.get_logger().warn(f'Simulation stopped: Cart exceeded limits at x={cart_x:.2f}m')

            self.cart_position.append(cart_x)
            self.angle_deviation.append(abs(theta))  # Track deviation magnitude

        except ValueError as e:
            self.get_logger().warn(f'Error processing joint state: {e}')

    def control_callback(self, msg):
        """Tracks control effort applied to the system."""
        if self.simulation_active:
            self.control_effort.append(abs(msg.data))

    def earthquake_callback(self, msg):
        """Tracks earthquake force applied to the cart."""
        if self.simulation_active:
            self.earthquake_force.append(abs(msg.data))

    def update_plot(self):
        """Real-time plot updates for performance metrics."""
        if not self.simulation_active:
            return  # Stop updating plots if simulation has stopped

        self.axs[0, 0].clear()
        self.axs[0, 0].plot(self.angle_deviation, label="Pole Angle Deviation (°)", color='red')
        self.axs[0, 0].set_title("Pole Angle Deviation")
        self.axs[0, 0].legend()

        self.axs[0, 1].clear()
        self.axs[0, 1].plot(self.cart_position, label="Cart Position (m)", color='blue')
        self.axs[0, 1].set_title("Cart Position")
        self.axs[0, 1].legend()

        self.axs[1, 0].clear()
        self.axs[1, 0].plot(self.control_effort, label="Control Force (N)", color='green')
        self.axs[1, 0].set_title("Control Effort")
        self.axs[1, 0].legend()

        self.axs[1, 1].clear()
        self.axs[1, 1].plot(self.earthquake_force, label="Earthquake Force (N)", color='orange')
        self.axs[1, 1].set_title("Earthquake Disturbance")
        self.axs[1, 1].legend()

        plt.tight_layout()
        plt.pause(0.01)  # Small delay for rendering update

def main(args=None):
    rclpy.init(args=args)
    node = PerformanceMonitor()
    try:
        plt.ion()  # Enable interactive mode for real-time plotting
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()  # Disable interactive mode
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
