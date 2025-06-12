from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Include Autoware main launch file (adjust path to your Autoware launch file)
    autoware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('autoware_launch_package'),  # Replace with actual Autoware launch package name
                'launch',
                'full_autoware_launch.py'  # Replace with actual main Autoware launch file
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
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
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        autoware_launch,
        autopilot_display,
        inference,
    ])
