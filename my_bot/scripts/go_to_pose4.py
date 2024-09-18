#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.duration import Duration
import time
import sys
import math
import paramiko

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles to quaternion.
    roll, pitch, yaw are in radians.
    """
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)

    return {'r1': qx, 'r2': qy, 'r3': qz, 'r4': qw}

# Contoh quaternion untuk berbagai orientasi
quaternion_right = euler_to_quaternion(0, 0, 0)  # Menghadap ke kanan (yaw = 0)
quaternion_up = euler_to_quaternion(0, 0, math.pi / 2)  # Menghadap ke atas (yaw = π/2)
quaternion_left = euler_to_quaternion(0, 0, math.pi)  # Menghadap ke kiri (yaw = π)
quaternion_down = euler_to_quaternion(0, 0, -math.pi / 2)  # Menghadap ke bawah (yaw = -π/2)

def play_sound_on_raspberry_pi(host, port, username, password):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(host, port, username, password)

    command = "/home/kelvinyxe/Desktop/robot_ws/src/my_bot/scripts/play_sound.sh"
    stdin, stdout, stderr = ssh.exec_command(command)
    print(stdout.read().decode())
    print(stderr.read().decode())

    ssh.close()

class GoToPose4(Node):

    def __init__(self):
        super().__init__('go_to_pose4')

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.goal_sent = False
        self.last_feedback_time = time.time()
        self.is_returning = False
        self.waypoints = [
            {'x': 0.900, 'y': -0.150, **quaternion_up},  # Menghadap ke atas
            {'x': 0.900, 'y': 1.864, **quaternion_right},  # Menghadap ke kanan
            {'x': 1.689, 'y': 1.864, **quaternion_down},  # Menghadap ke bawah
            {'x': 1.689, 'y': 1.184, **quaternion_right},  # Menghadap ke kanan
            {'x': 2.048, 'y': 1.184, **quaternion_down},  # Menghadap ke kanan
            {'x': 2.048, 'y': 0.477, **quaternion_down},  # Menghadap ke kanan
            {'x': 1.988, 'y': -0.113, **quaternion_left},  # Menghadap ke atas
            {'x': 1.988, 'y': -0.113, **quaternion_left},  # Menghadap ke atas
            {'x': 1.988, 'y': -0.079, **quaternion_left},  # Menghadap ke atas
            {'x': 1.440, 'y': -0.020, **quaternion_right},  # Menghadap ke kiri
        ]
        self.return_waypoints = [
            {'x': 1.988, 'y': -0.079, **quaternion_right},  # Menghadap ke atas
            {'x': 1.988, 'y': -0.113, **quaternion_up},  # Menghadap ke kanan
            {'x': 2.048, 'y': 0.477, **quaternion_up},  # Menghadap ke kanan
            {'x': 2.048, 'y': 1.184, **quaternion_left},  # Menghadap ke kanan
            {'x': 1.689, 'y': 1.184, **quaternion_up},  # Menghadap ke kiri
            {'x': 1.689, 'y': 1.864, **quaternion_left},  # Menghadap ke atas
            {'x': 0.900, 'y': 1.864, **quaternion_down},  # Menghadap ke kiri
            {'x': 0.900, 'y': -0.150, **quaternion_left},  # Menghadap ke bawah
        ]
        self.start_position = {'x': -0.028, 'y': 0.014, **quaternion_right}  # Menghadap ke kanan
        self.current_waypoint_index = 0
        self.send_goal(self.waypoints[self.current_waypoint_index])  # Mulai dengan waypoint pertama

    def send_goal(self, pos):
        self.goal_sent = True

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = pos['x']
        goal_msg.pose.pose.position.y = pos['y']
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = pos['r1']
        goal_msg.pose.pose.orientation.y = pos['r2']
        goal_msg.pose.pose.orientation.z = pos['r3']
        goal_msg.pose.pose.orientation.w = pos['r4']

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.goal_sent = False
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.goal_sent = False
        if result:
            if self.is_returning:
                self.get_logger().info('Goal succeeded!')
                self.current_waypoint_index += 1
                if self.current_waypoint_index < len(self.return_waypoints):
                    next_waypoint = self.return_waypoints[self.current_waypoint_index]
                    self.send_goal(next_waypoint)
                else:
                    self.send_goal(self.start_position)
                    self.get_logger().info('Success return to start position!')
                    rclpy.shutdown()
            else:
                self.get_logger().info('Goal succeeded!')
                self.current_waypoint_index += 1
                if self.current_waypoint_index < len(self.waypoints):
                    next_waypoint = self.waypoints[self.current_waypoint_index]
                    self.send_goal(next_waypoint)
                else:
                    self.play_sound_and_return_to_start()
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(future.result().status))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_time = time.time()
        if current_time - self.last_feedback_time >= 15:
            self.get_logger().info('Received feedback: {0}'.format(feedback))
            self.last_feedback_time = current_time

    def return_to_start(self):
        self.get_logger().info('Returning to start position...')
        self.is_returning = True
        self.current_waypoint_index = 0  # Set index to the first return waypoint
        next_waypoint = self.return_waypoints[self.current_waypoint_index]
        self.send_goal(next_waypoint)

    def play_sound_and_return_to_start(self):
        self.get_logger().info('Playing sound...')
        raspberry_pi_ip = "172.20.10.3"
        ssh_port = 22
        ssh_username = "kelvinyxe"
        ssh_password = "Nayonaise23"
        play_sound_on_raspberry_pi(raspberry_pi_ip, ssh_port, ssh_username, ssh_password)
        self.return_to_start()

def main(args=None):
    rclpy.init(args=args)
    navigator = GoToPose4()

    try:
        rclpy.spin(navigator)

    except KeyboardInterrupt:
        navigator.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
