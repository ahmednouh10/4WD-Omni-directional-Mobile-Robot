import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'mecanum_bot' # CHANGE THIS IF YOUR PACKAGE NAME IS DIFFERENT
    pkg_share = get_package_share_directory(pkg_name)
    
    # 1. Process the URDF (Xacro)
    xacro_file = os.path.join(pkg_share, 'urdf', 'mecanum_robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])
    
    # 2. Launch Gazebo (Empty World)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 3. Spawn the Robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_mecanum_robot',
                   '-z', '0.1'], # Spawn slightly above ground
        output='screen'
    )

    # 4. Robot State Publisher (Publishes TFs)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])