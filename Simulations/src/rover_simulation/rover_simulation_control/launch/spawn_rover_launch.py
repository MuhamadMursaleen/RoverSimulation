"""
Author:
- Muhammad Mursaleen @ Tampere University
- muhammadmursaleen09@gmail.com

Description:
- this Luanch file will run the digital twin in empty gazebo enviornment
- spawn the the rover robot in gazevo with default gazebo plugin skid_steer_drive
- add the rover defination in robot_state_publisher

"""
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    """
    This function is responisble to run thses nodes 
        - spawn_entity in gazebo
        - robot_state_publisher
        - joint_state_publisher
    """

    # Define paths for package and files
    pkg_share = launch_ros.substitutions.FindPackageShare(package='rover_description').find('rover_description')
    default_model_path = os.path.join(pkg_share, 'urdf/wheel_rps_controller_drive.xacro')

    # Define the node for robot_state_publisher_node
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    # Define the node for joint_state_publisher_node
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Define the node for spawn_entity in gazebo
    spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'rover', '-topic', 'robot_description'],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='model',
            default_value=default_model_path,
            description='Absolute path to robot URDF file'
        ),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
    ])
