from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        Node(
            package='my_bot',
            executable='go_to_pose3.py',
            name='go_to_pose',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
        )
    ])
