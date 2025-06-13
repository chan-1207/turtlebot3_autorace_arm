#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time

class VelocityCommandPublisher(Node):
    def __init__(self):
        super().__init__('velocity_command_publisher')
        
        # Create publisher for velocity commands
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands',
            10)
        
        # Create timer for publishing velocity commands
        self.timer = self.create_timer(1.0, self.publish_velocity_command)
        self.get_logger().info('Velocity Command Publisher node has been started')
        
        # Initialize state for alternating velocities
        self.use_positive_velocity = True

    def publish_velocity_command(self):
        # Create velocity command message
        velocity_msg = Float64MultiArray()
        
        # Alternate between 1.0 and -1.0 rad/s
        velocity = 1.0 if self.use_positive_velocity else -1.0
        velocity_msg.data = [velocity, velocity, velocity]  # For all three joints
        
        # Toggle the state for next iteration
        self.use_positive_velocity = not self.use_positive_velocity
        
        # Publish velocity command
        self.publisher.publish(velocity_msg)
        self.get_logger().info(f'Published velocity command: {velocity} rad/s')

def main(args=None):
    rclpy.init(args=args)
    node = VelocityCommandPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 