# lidar_bve_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bird_view',
            executable='lidar_subscriber',
            name='lidar_subscriber',
            output='screen'
        ),
        Node(
            package='bird_view',
            executable='lidar_bird_view',
            name='lidar_bird_view',
            output='screen'
        )
    ])
