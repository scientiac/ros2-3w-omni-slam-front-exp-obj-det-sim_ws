import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from pathlib import Path

PACKAGE_NAME = "ros2_omni_robot_sim"

ARGUMENTS = [
    DeclareLaunchArgument('world', 
                          default_value="sorting-round",
                          description='Gazebo World'),
    DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),
]

def generate_launch_description():
    robot_model = "3w_v2"
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Source Environment (Need it to be able find mesh files)
    pkg_path = get_package_share_directory(PACKAGE_NAME)
    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            str(Path(pkg_path).parent.resolve()), ":",
             os.path.join(pkg_path, 'worlds'),
            ]
    )

    # Create a robot_state_publisher node
    xacro_file = os.path.join(pkg_path,'urdf', robot_model, 'main.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])
    
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # launch gazebo
    gazebo_launch_path = PathJoinSubstitution([
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
            ])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_path]),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'),
                         '.sdf',
                          ' -r',
                          ' -v 4'])
        ]
    )

    # Spawn the robot in Gazebo
    spawn_robot = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', robot_model,
                                  '-x', '-1.3',  # 4th Corner X
                                  '-y', '-1.3',  # 4th Corner Y
                                  '-z', '0.1'],  # Height
                        output='screen')
    
    # gz bridge 
    bridge_params = os.path.join(pkg_path,'config', 'gz_bridge', f'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{'config_file': bridge_params}],
    )

    
    # spawn controller 
    # For 3w_v2, we have 3 wheels and a camera servo (assuming camera servo is still relevant or if it was part of 3w)
    # Checking 3w_v2 urdf, it has ros2_control.urdf.xacro.
    # Let's assume the controller config for 3w_v2 defines these controllers.
    spawn_wheel_controller = Node(package='controller_manager', executable='spawner',
                        arguments=['joint_state_broadcaster', 
                                    'wheel1_controller', 
                                    'wheel2_controller', 
                                    'wheel3_controller'],
                        output='screen')

    kinematics = Node(
        package=PACKAGE_NAME,
        executable="kinematics",
        parameters=[{"use_sim_time": use_sim_time}]
    )

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_resource_path)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(gazebo)
    ld.add_action(spawn_robot)
    ld.add_action(ros_gz_bridge)
    ld.add_action(spawn_wheel_controller)
    ld.add_action(kinematics)
    return ld
