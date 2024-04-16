"""
Author:
- Muhammad Mursaleen @ Tampere University
- muhammadmursaleen09@gmail.com

Description: 
- By launching it you can run the simulation and can move it by joy stick ps4
- this Luanch file will run the gazebo envoirnment
- spawn the the rover robot in gazebo envoirment in custom world with 
    custom skid steer drive 
- add the rover defination in robot_state_publisher
- Activate the ros2 contoller: velocity_controller
- Activate the ros2 contoller: joint_state_broadcaster
- run the kmodel node
- run the joy_stick node
- run the cmd_publisher node, inputs are command velocity of motors 
    from -1000 to 1000, instaed of radian per second

"""
import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import  ExecuteProcess, TimerAction
def generate_launch_description():
    """
    This function is responisble to run these nodes 
        - kmodel node
        - joy_stick node
        - cmd_publisher node
    Also it will activate following ros2 controller
        - velocity_controller
        - joint_state_broadcaster
    It will also spawn robot in gazebo world
    """
    
    # Define paths for package and files
    world_pkg_share = launch_ros.substitutions.FindPackageShare(package='rover_simulation_control').find('rover_simulation_control')
    world_file_path = os.path.join(world_pkg_share, 'worlds', 'rover_world.world')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    launch_file_dir =  os.path.join(world_pkg_share, 'launch')
    
    # Activate the ros2 contoller: velocity_controller
    forward_velocity_controller = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'velocity_controller'], output='screen'
        )

    # Activate the ros2 contoller: joint_state_broadcaster
    joint_state_broadcaster = ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'], output='screen'
        )
    
    # kmodel node
    kmodel_control_interface = launch_ros.actions.Node(
        package='kinematic_model_interface',
        executable='kinematics_model_interface_node',
        name='kinematics_model_interface_node'
    )
    # joy stick ps4 kmodel node
    kmodel_ps4_joystick = launch_ros.actions.Node(
        package='remote_joystick_control',
        executable='ps4joystick_kmodel_cmd_node',
        name='ps4joystick_kmodel_cmd_node'
    )

    # Command velocity publisher for motor driver
    cmd_publisher = launch_ros.actions.Node(
        package='rover_simulation_control',
        executable='cmd_publisher_controller',
        name='cmd_publisher_controller'
    )

    # Make some delayed based timer action so these nodes will be runned after 
    # some delay
    delayed_forward_velocity_controller = TimerAction(
        period=5.0, actions=[forward_velocity_controller])
    
    delayed_joint_state_broadcaster = TimerAction(
        period=7.0, actions=[joint_state_broadcaster])
    

    delayed_kmodel_control_interface = TimerAction(
        period=10.0, actions=[kmodel_control_interface])
    
    delayed_kmodel_ps4_joystick = TimerAction(
        period=12.0, actions=[kmodel_ps4_joystick])
    
    delayed_cmd_publisher = TimerAction(period=14.0, actions=[cmd_publisher])
    
    spawn_rover = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_rover_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return launch.LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            name='world',
            default_value=world_file_path,
            description='Absolute path to Gazebo world file'
        ),
        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', LaunchConfiguration('world')],
            output='screen'
        ),
        spawn_rover, 
        delayed_joint_state_broadcaster,
        delayed_forward_velocity_controller,
        delayed_kmodel_control_interface,
        delayed_kmodel_ps4_joystick,
        delayed_cmd_publisher,

    ])
