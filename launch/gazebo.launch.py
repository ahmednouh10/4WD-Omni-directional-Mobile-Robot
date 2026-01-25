import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_bot')
    
    # Path to the Xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'mecanum_robot.urdf.xacro')
    
    # Path to the RViz configuration file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'mecanum_bot.rviz')

    # Process the Xacro file into a URDF
    robot_description_config = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': robot_description_config}

    # Start Gazebo server
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py'))
    )

    # Start Gazebo client
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py'))
    )

    # Node to spawn the robot in Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mecanum_bot', '-topic', 'robot_description', '-z', '0.05'],
        output='screen'
    )

    # Node to publish the robot's state
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )
    
    # RViz2 Node (optional, for visualization alongside Gazebo)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot,
        robot_state_publisher_node,
        rviz_node
    ])