# rover_simulation_control

This package encompasses all definitions of the rover robot model for simulation.

## Directories:
1. **launch:** It contains all launch files for the model description.
2. **rover_simulation_control:** It contains nodes to run the robot.
3. **rosbags:** It contains rosbags from real-time and simulation sensors.

## Dependencies:

- rover_description
- direct_drive_interface
- kinematic_model_interface
- remote_joystick_control
- rover_custom_msgs

## Launch files:

### digital_twin_launch.py
To execute the digital twin in Gazebo, it requires linear and angular velocities from the actual robot. It utilizes the default Gazebo plugin for skid steer drive.

```bash
$ ros2 launch rover_simulation_control digital_twin_launch.py
```

### display_rviz_launch.py
This file displays the model in RViz.

```bash
$ ros2 launch rover_simulation_control display_rviz_launch.py 
```

### gazebo_cmd_ddrive_setup_launch.py
To launch simulation in Gazebo and allow teleoperation via a PS4 controller using direct drive. It will publish the following nodes:

- joint_state_publisher_node
- robot_state_publisher_node
- spawn_entity
- joint_state_broadcaster
- forward_velocity_controller
- ddrive_control_interface
- ddrive_ps4_joystick
- cmd_publisher

```bash
$ ros2 launch rover_simulation_control gazebo_cmd_ddrive_setup_launch.py 
```

### gazebo_cmd_kmodel_setup_launch.py
To launch simulation in Gazebo and allow teleoperation via a PS4 controller using kmodel drive. It will publish the following nodes:

- joint_state_publisher_node
- robot_state_publisher_node
- spawn_entity
- joint_state_broadcaster
- forward_velocity_controller
- kmodel_control_interface
- kmodel_ps4_joystick
- cmd_publisher

```bash
$ ros2 launch rover_simulation_control gazebo_cmd_kmodel_setup_launch.py 
```

### rover_world_launch.py
To launch simulation in Gazebo custom world and allow teleoperation via a PS4 controller using kmodel drive. It will publish the following nodes:

- joint_state_publisher_node
- robot_state_publisher_node
- spawn_entity
- joint_state_broadcaster
- forward_velocity_controller
- kmodel_control_interface
- kmodel_ps4_joystick
- cmd_publisher

```bash
$ ros2 launch rover_simulation_control rover_world_launch.py 
```

### spawn_rover_launch.py
To spawn the robot in Gazebo without any teleoperated node. It will use a custom velocity controller plugin for each wheel. It will publish the following nodes:

- joint_state_publisher_node
- robot_state_publisher_node
- spawn_entity


```bash
$ ros2 launch rover_simulation_control spawn_rover_launch.py 
```

### spawn_rps_drive_rover_launch.py
To launch simulation in Gazebo without any teleoperated node. It will use a custom velocity controller plugin for each wheel. It will publish the following nodes:

- joint_state_publisher_node
- robot_state_publisher_node
- spawn_entity
- joint_state_broadcaster
- forward_velocity_controller

```bash
$ ros2 launch rover_simulation_control spawn_rps_drive_rover_launch.py 
```

### spawn_skid_steer_launch.py
To launch simulation in Gazebo without any teleoperated node. It will use the default Gazebo skid_steer_drive plugin. It will publish the following nodes:

- joint_state_publisher_node
- robot_state_publisher_node
- spawn_entity

```bash
$ ros2 launch rover_simulation_control spawn_skid_steer_launch.py 
```

### test_launch.py
this Luanch file is a rough test launch file you can modify any code here
instead of modifying the original file.


```bash
$ ros2 launch rover_simulation_control test_launch.py 
```

## rover_simulation_control files:

This directory contains all necessary nodes for simulation, including the following:

### cmd_publisher_controller.py:
It uses the same input as being used for the motor controller (-1000 to 1000) to drive the robot with a custom velocity controller.

```bash
$ ros2 run rover_simulation_control cmd_publisher_controller.py 
```

### rps_publisher_controller.py:
It uses the RPS as input to drive the robot with a custom velocity controller.

```bash
$ ros2 run rover_simulation_control rps_publisher_controller.py 
```

### digital_twin_node.py:
This node takes input from the real robot in the form of linear_x velocity and angular_z velocity from the topic "/camera/pose/sample" and publishes it to the simulator model.

```bash
$ ros2 run rover_simulation_control digital_twin_node.py 
```