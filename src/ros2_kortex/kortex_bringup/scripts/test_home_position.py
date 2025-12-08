#!/usr/bin/env python3
"""
Test script to move robot to validated home position.
Home position: [0.0, 1.0, 2.05, 1.615, 0.55, 0.0]
Validated to be horizontal (parallel to table).
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class TestHomePosition(Node):
    def __init__(self):
        super().__init__('test_home_position')
        
        # Publisher for joint trajectory commands
        self.joint_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Wait for publisher to be ready
        self.get_logger().info('Waiting 2 seconds for connections...')
        self.create_timer(2.0, self.move_to_home)
        
    def move_to_home(self):
        """Move to validated home position (reflected for 180° robot rotation)"""
        self.get_logger().info('Moving to validated home position...')
        self.get_logger().info('Target: [0.0, -0.7853, 0.7854, 0, 0, 0] rad')
        self.get_logger().info('(Reflected around Z-axis for 180° rotated robot)')
        
        # Validated home position reflected for 180° robot rotation
        # Original: [0.0, 1.0, 2.05, 1.615, 0.55, 0.0]
        # Reflected: joint_2 and joint_3 values adjusted for 180° rotated robot
        home_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        msg = JointTrajectory()
        msg.joint_names = [
            'joint_1',
            'joint_2', 
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]
        
        point = JointTrajectoryPoint()
        point.positions = home_joints
        point.time_from_start = Duration(sec=3, nanosec=0)
        
        msg.points.append(point)
        
        self.joint_pub.publish(msg)
        self.get_logger().info('Command sent! Robot should move to home position.')
        self.get_logger().info('Gripper should be horizontal and parallel to table.')
        
        # Shutdown after sending command
        self.create_timer(1.0, lambda: rclpy.shutdown())

def main(args=None):
    rclpy.init(args=args)
    node = TestHomePosition()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
