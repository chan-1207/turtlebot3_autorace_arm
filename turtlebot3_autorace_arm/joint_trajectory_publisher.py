#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import time

class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')

        # Create publisher for joint trajectory
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)

        # Create timer for publishing trajectory
        self.timer = self.create_timer(2.0, self.publish_trajectory)
        self.get_logger().info('Joint Trajectory Publisher node has been started')

        # Initialize state for alternating positions
        self.use_positive_position = True

    def publish_trajectory(self):
        # Create trajectory message
        trajectory_msg = JointTrajectory()

        # Set joint names
        trajectory_msg.joint_names = ['joint1']

        # Create trajectory point
        point = JointTrajectoryPoint()
        # Alternate between -0.3 and 0.3
        position = 1.0 if self.use_positive_position else -1.0
        point.positions = [position]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 0

        # Toggle the state for next iteration
        self.use_positive_position = not self.use_positive_position

        # Add point to trajectory
        trajectory_msg.points.append(point)

        # Publish trajectory
        self.publisher.publish(trajectory_msg)
        self.get_logger().info(f'Published joint trajectory with position {position}')

def main(args=None):
    rclpy.init(args=args)
    node = JointTrajectoryPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()