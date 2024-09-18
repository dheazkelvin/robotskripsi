#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf_transformations import quaternion_from_euler
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class PathPlotter(Node):

    def __init__(self):
        super().__init__('path_plotter')
        self.xAnt = 0.0
        self.yAnt = 0.0
        self.cont = 0

        self.declare_parameter('max_list_append', 1000)
        self.max_append = self.get_parameter('max_list_append').get_parameter_value().integer_value

        if self.max_append <= 0:
            self.get_logger().warn('The parameter max_list_append is not correct')

        self.pub = self.create_publisher(Path, '/path', 1)
        self.path = Path()

        self.sub = self.create_subscription(Odometry, '/odom', self.callback, 1)

        self.timer = self.create_timer(1.0 / 900.0, self.publish_path)

    def callback(self, data):
        pose = PoseStamped()
        pose.header.frame_id = "odom"
        pose.pose.position.x = float(data.pose.pose.position.x)
        pose.pose.position.y = float(data.pose.pose.position.y)
        pose.pose.orientation.x = float(data.pose.pose.orientation.x)
        pose.pose.orientation.y = float(data.pose.pose.orientation.y)
        pose.pose.orientation.z = float(data.pose.pose.orientation.z)
        pose.pose.orientation.w = float(data.pose.pose.orientation.w)

        if (self.xAnt != pose.pose.position.x and self.yAnt != pose.pose.position.y):
            self.path.header.frame_id = "odom"
            self.path.header.stamp = self.get_clock().now().to_msg()
            pose.header.stamp = self.path.header.stamp
            self.path.poses.append(pose)

            self.cont += 1

            if self.cont > self.max_append:
                self.path.poses.pop(0)

            self.pub.publish(self.path)

        self.xAnt = pose.pose.position.x
        self.yAnt = pose.pose.position.y

    def publish_path(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    path_plotter = PathPlotter()

    try:
        rclpy.spin(path_plotter)
    except KeyboardInterrupt:
        pass
    finally:
        path_plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
