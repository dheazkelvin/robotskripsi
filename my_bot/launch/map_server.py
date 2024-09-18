import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Create LaunchDescription object
    ld = LaunchDescription()

    # Define the path to the map server configuration file
    map_server_config_path = '/home/ubuntu/Desktop/dev_ws/map_skripsi.yaml'

    # Define the map server node
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_server_config_path}]
    )

    # Define lifecycle nodes and their parameters
    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True

    # Define the lifecycle manager node
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,  # Ensures proper tty emulation for output
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}]
    )

    # Add actions to the launch description
    ld.add_action(map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld
