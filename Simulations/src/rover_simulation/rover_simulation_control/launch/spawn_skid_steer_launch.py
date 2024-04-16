"""
Author:
- Muhammad Mursaleen @ Tampere University
- muhammadmursaleen09@gmail.com

Description:
- this Luanch file will run the gazebo envoirnment
- spawn the the rover robot in gazebo envoirment with default 
    gazebo skid_steer_drive_plugin
- add the rover defination in robot_state_publisher

"""
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():

    """
    This function is responisble to run three nodes 
        - spawn_entity in gazebo
        - robot_state_publisher
        - joint_state_publisher
    
    """

    # get the package location for rover_description
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='rover_description').find('rover_description')
    
    # get the loaction of xacro file in to the package
    default_model_path = os.path.join(pkg_share, 'urdf/skid_steer_drive.xacro')

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
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
       
    ])