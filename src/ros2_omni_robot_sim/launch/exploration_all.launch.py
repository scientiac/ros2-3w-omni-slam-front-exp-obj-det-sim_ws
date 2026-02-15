import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_omni = get_package_share_directory('ros2_omni_robot_sim')
    pkg_explore = get_package_share_directory('ros2_exploration')

    world = LaunchConfiguration('world', default='sorting-round')

    # SLAM Toolbox (Online Async)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_omni, 'launch', 'slam_gazebo_sim.launch.py')]),
        launch_arguments={'world': world}.items()
    )

    # Nav2 (Navigation Stack)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_omni, 'launch', 'nav2.launch.py')]),
        launch_arguments={'world': world}.items()
    )

    # Frontier Exploration + Twist Converter
    exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_explore, 'launch', 'exploration_launch.py')]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # --- NEW: Object Detection Node ---
    vision_system = Node(
        package='ros2_vision_system',
        executable='object_detection',
        name='object_detection_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='sorting-round'),
        
        # Start SLAM immediately (it usually handles the Gazebo start)
        slam,
        
        # Delay Nav2 to ensure map is being published
        TimerAction(period=10.0, actions=[nav2]),
        
        # Start exploration last
        TimerAction(period=15.0, actions=[exploration]),

        # Delay Vision System to ensure Gazebo is rendering
        TimerAction(period=5.0, actions=[vision_system])
        
    ])
