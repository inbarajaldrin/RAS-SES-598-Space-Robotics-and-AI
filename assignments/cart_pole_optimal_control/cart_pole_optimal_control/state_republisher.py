#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class StateRepublisher(Node):
    def __init__(self):
        super().__init__('state_republisher')
        
        # Create QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to Gazebo joint states
        self.gz_joint_sub = self.create_subscription(
            JointState,
            '/world/empty/model/cart_pole/joint_state',
            self.joint_state_callback,
            sensor_qos
        )
        
        # Publisher for ROS2 joint states
        self.joint_pub = self.create_publisher(
            JointState,
            'joint_states',  # Standard ROS2 joint states topic
            reliable_qos
        )
        
        self.get_logger().info('State Republisher node started')
        
    def joint_state_callback(self, msg):
        # Create new joint state message
        new_msg = JointState()
        new_msg.header = msg.header
        new_msg.header.frame_id = 'world'  # Set proper frame_id
        
        # Copy joint states but ensure correct order and naming
        new_msg.name = ['cart_to_base', 'pole_joint']  # Fixed order of joints
        new_msg.position = [0.0] * 2
        new_msg.velocity = [0.0] * 2
        new_msg.effort = [0.0] * 2
        
        # Map received joint states to correct indices
        for i, name in enumerate(msg.name):
            if name == 'cart_to_base':
                idx = 0
            elif name == 'pole_joint':
                idx = 1
            else:
                continue
                
            if i < len(msg.position):
                new_msg.position[idx] = msg.position[i]
            if i < len(msg.velocity):
                new_msg.velocity[idx] = msg.velocity[i]
            if i < len(msg.effort):
                new_msg.effort[idx] = msg.effort[i]
        
        # Publish the reformatted joint states
        self.joint_pub.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StateRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 