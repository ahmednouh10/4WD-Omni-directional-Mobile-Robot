import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mecanum_bot')
    
    # Path to the Xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'mecanum_robot.urdf.xacro')
    
    # Path to the RViz configuration file (we'll create this next)
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'mecanum_bot.rviz')

    # Process the Xacro file into a URDF
    robot_description_config = Command(['xacro ', xacro_file])
    robot_description = {'robot_description': robot_description_config}

    # Node to publish the robot's state
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Node to publish joint states (with GUI for manual control)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])