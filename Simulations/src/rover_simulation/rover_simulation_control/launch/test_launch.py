"""
Author:
- Muhammad Mursaleen @ Tampere University
- muhammadmursaleen09@gmail.com

Description:
- this Luanch file is a rough test launch file you can modify any code here
instead of modifying the original file.

"""
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='rover_description').find('rover_description')
    default_model_path = os.path.join(pkg_share, 'urdf/skid_steer_drive.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/config.rviz')
    world_file_path = os.path.join(pkg_share, 'worlds', 'rover_world.world')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    digital_twin_node = launch_ros.actions.Node(
        package='rover_simulation_control',
        executable='digital_twin_node',
        name='digital_twin_node',
        output='screen',
    )

    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'rover', '-topic', 'robot_description', '-world', LaunchConfiguration('world')],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot URDF file'
        ),
        launch.actions.DeclareLaunchArgument(
            name='world',
            default_value=world_file_path,
            description='Absolute path to Gazebo world file'
        ),
        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', LaunchConfiguration('world')],
            output='screen'
        ),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        rviz_node,
        digital_twin_node
    ])
