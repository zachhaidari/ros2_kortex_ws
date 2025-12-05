#!/usr/bin/env python3
"""Echo N messages from a `sensor_msgs/msg/JointState` topic then exit.

Usage:
  python3 tools/echo_n_joint_states.py /joint_states 5

This is a small convenience helper for demos where `ros2 topic echo`'s
`--limit` flag isn't available in the ROS2 distro in use.
"""
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class EchoNJointStates(Node):
    def __init__(self, topic: str, n: int):
        super().__init__('echo_n_joint_states')
        self._count = 0
        self._target = max(1, int(n))
        self.get_logger().info(f'Subscribing to "{topic}" and collecting {self._target} messages')
        self._sub = self.create_subscription(JointState, topic, self._cb, 10)

    def _cb(self, msg: JointState):
        self._count += 1
        # Print a compact YAML-like representation for readability
        names = list(msg.name)
        positions = list(msg.position)
        velocities = list(msg.velocity) if msg.velocity else []
        efforts = list(msg.effort) if msg.effort else []
        print(f"--- message #{self._count}")
        print(f"names: {names}")
        print(f"positions: {positions}")
        if velocities:
            print(f"velocities: {velocities}")
        if efforts:
            print(f"efforts: {efforts}")
        if self._count >= self._target:
            self.get_logger().info(f'Received {self._target} messages, shutting down')
            rclpy.shutdown()


def main(argv=None):
    argv = argv or sys.argv[1:]
    if len(argv) < 2:
        print("Usage: python3 tools/echo_n_joint_states.py /joint_states 5")
        return 1
    topic = argv[0]
    try:
        n = int(argv[1])
    except ValueError:
        print("Second argument must be an integer number of messages to collect")
        return 1

    rclpy.init()
    node = None
    try:
        node = EchoNJointStates(topic, n)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
