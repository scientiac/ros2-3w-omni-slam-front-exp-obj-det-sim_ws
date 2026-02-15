import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Get the value of 'use_sim_time', the default is 'false'
    # This value must be consistent with Gazebo and Nav2
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        # Declare the 'use_sim_time' parameter
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Start the type conversion node
        Node(
            package='ros2_exploration',
            executable='twist_converter',
            name='cmd_vel_type_converter',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Start the exploration node
        Node(
            package='ros2_exploration',
            executable='exploration_node',
            name='exploration_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
