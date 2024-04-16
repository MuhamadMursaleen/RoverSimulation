"""
Author:
- Muhammad Mursaleen @ Tampere University
- muhammadmursaleen09@gmail.com

Description:
- this Luanch file will run the rviz
- spawn the the rover robot in rviz with default gazebo plugin
- add the rover defination in robot_state_publisher

"""
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():

    """
    This function is responisble to run three nodes 
        - spawn_entity in rviz
        - robot_state_publisher
        - joint_state_publisher
    
    Also it will activate following ros2 controller
        - velocity_controller
        - joint_state_broadcaster
    """
    # get the package location for rover_description
    pkg_share = launch_ros.substitutions.FindPackageShare(
        package='rover_description').find('rover_description')
    
    # get the loaction of xacro file in to the package   
    default_model_path = os.path.join(pkg_share, 'urdf/skid_steer_drive.xacro')
    
    # Define the node for robot_state_publisher_node
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/config.rviz')

    # Define the node for robot_state_publisher_node
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(
            ['xacro ', LaunchConfiguration('model')])}]
    )

    # Define the node for joint_state_publisher_node
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    # Define the node for spawn_entity in rviz
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', 
                                            default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', 
                                             default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])