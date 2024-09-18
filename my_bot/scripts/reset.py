#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

def reset_pose(pose):
    # Reset position
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = 0.0

    # Reset orientation (quaternion)
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0  # Represents no rotation

def main(args=None):
    rclpy.init(args=args)
    node = Node('reset_pose_node')

    my_pose = Pose()
    reset_pose(my_pose)

    # Now my_pose is reset to default values
    # You can publish it or use it as needed

    rclpy.shutdown()

if __name__ == '__main__':
    main()
