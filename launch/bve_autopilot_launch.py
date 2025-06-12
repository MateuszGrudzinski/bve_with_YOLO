from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    lidar_sub = Node(
        package='bird_view',
        executable='lidar_subscriber',
        name='lidar_subscriber',
        output='screen',
        parameters=[],
        # Adjust QoS or remappings here if needed
    )

    autopilot_display = Node(
        package='bird_view',
        executable='autopilot_display',
        name='autopilot_display',
        output='screen',
    )

    inference = Node(
        package='bird_view',
        executable='camera_lidar_fusion',
        name='camera_lidar_fusion',
        output='screen'
    )
    return LaunchDescription([
        lidar_sub,
        autopilot_display,
        inference,
    ])
