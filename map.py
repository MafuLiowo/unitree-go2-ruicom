from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{
                'yaml_filename': '/home/unitree/ai-unitree-go2-ruicom/map/map4.yaml',
                'use_sim_time': False
            }],
            arguments=['--ros-args', '--log-level', 'info'],
            output='screen'
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{
                'node_names': ['map_server'],
                'autostart': True,
                'bond_timeout': 0.0
            }],
            output='screen'
        )
    ])