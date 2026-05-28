from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.28945', '0', '-0.046825', '0', '2.8782', '0', 'base_link', 'utlidar_frame'],
            name='static_tf_publisher'
        ),

        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/utlidar/cloud_base'),   
                ('scan', '/scan')                     
            ],
            parameters=[{
                'target_frame': 'base_link',           
                'transform_tolerance': 0.01,          
                'min_height': -0.2,                    
                'max_height': 0.8,                    
                'angle_min': -3.141592654,             
                'angle_max': 3.141592654,              
                'angle_increment': 0.003141592,        
                'scan_time': 0.03333,                 
                'range_min': 0.1,                      
                'range_max': 3.0,                      
                'use_inf': True,                       
                'inf_epsilon': 1.0
            }]
        )
    ])