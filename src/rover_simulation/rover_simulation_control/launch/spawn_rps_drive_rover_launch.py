"""
Author:
- Muhammad Mursaleen @ Tampere University
- muhammadmursaleen09@gmail.com

Description:
- this Luanch file will run the gazebo envoirnment
- spawn the the rover robot in gazebo envoirment with 
    custom skid steer drive 
- add the rover defination in robot_state_publisher
- Activate the ros2 contoller: velocity_controller
- Activate the ros2 contoller: joint_state_broadcaster

"""
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch.actions import  ExecuteProcess, TimerAction

def generate_launch_description():

    """
    This function is responisble to run three nodes 
        - spawn_entity in gazebo
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
    default_model_path = os.path.join(
        pkg_share, 'urdf/wheel_rps_controller_drive.xacro')

    # Define the node for robot_state_publisher_node
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(
            ['xacro ', LaunchConfiguration('model')])}]
    )
    
    # Define the node for joint_state_publisher_node
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Activate the ros2 contoller: velocity_controller
    forward_velocity_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
                 'velocity_controller'], output='screen'
        )

    # Activate the ros2 contoller: joint_state_broadcaster
    joint_state_broadcaster = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
                 'joint_state_broadcaster'], output='screen'
        )
    
    # Define the node for spawn_entity in gazebo
    spawn_entity = launch_ros.actions.Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=['-entity', 'rover', '-topic', 'robot_description'],
    output='screen'
    )

    # Make some delayed based timer action so these nodes will be runned after 
    # some delay
    delayed_forward_velocity_controller = TimerAction(
        period=5.0, actions=[forward_velocity_controller])

    delayed_joint_state_broadcaster = TimerAction(
        period=5.0, actions=[joint_state_broadcaster])

    delayed_spawn_entity = TimerAction(period=5.0, actions=[spawn_entity])

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', 
                                            default_value=default_model_path,
                                            description='Absolute path to urdf file'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 
                                           'libgazebo_ros_init.so', '-s', 
                                           'libgazebo_ros_factory.so'],
                                            output='screen'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        delayed_spawn_entity,
        delayed_joint_state_broadcaster,
        delayed_forward_velocity_controller,
       
    ])