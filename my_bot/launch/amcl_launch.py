from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the launch directory
    launch_dir = os.path.join(get_package_share_directory('my_bot'), 'launch')

    return LaunchDescription([
        # Use simulation time
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        # Lifecycle manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': True,
                         'autostart': True,
                         'node_names': ['amcl']}]
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
