#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from rclpy.qos import QoSProfile, ReliabilityPolicy

class QoSBridge(Node):
    def __init__(self):
        super().__init__('qos_bridge')
        # QoS profile for BEST_EFFORT
        best_effort_qos = QoSProfile(depth=10)
        best_effort_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        # QoS profile for RELIABLE
        reliable_qos = QoSProfile(depth=10)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE

        # Subscriber with BEST_EFFORT QoS
        self.subscriber_ = self.create_subscription(
            PoseArray,
            'pose_array_topic',
            self.listener_callback,
            best_effort_qos)

        # Publisher with RELIABLE QoS
        self.publisher_ = self.create_publisher(
            PoseArray,
            'pose_array_topic_reliable',
            reliable_qos)

    def listener_callback(self, msg):
        # Republish the received message
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = QoSBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
