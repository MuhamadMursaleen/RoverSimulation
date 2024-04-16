
## Robot Model Details

The robot model has been meticulously constructed using xacro files, incorporating approximate measurements for its size. Due to the unavailability of exact weights for all links, estimated weights have been assigned to facilitate inertial measurements. To ensure versatility and ease of future development, individual xacro files have been created, thoroughly tested in both RViz and Gazebo simulations.

### Breakdown of Xacro Files:

1. **Inertia Parameters:** Encompasses all definitions essential for inertial measurements.

2. **Intel RealSense T265:** Includes support links and plugins specifically for the Intel RealSense T265. Presently, only the depth sensor is integrated.

3. **Materials:** Houses all color definitions necessary for Gazebo simulation.

4. **Plugin Skid Steer Drive:** Utilizes the default Gazebo plugin for skid steer drive.

5. **Properties:** Contains size definitions for all links within the rover model.

6. **RoboSense Bpearl:** Provides link definitions to support the RoboSense Bpearl sensor. The sensor plugin has been modified based on the original Bpearl properties. As the manufacturer did not offer a simulation for this sensor, the Velodyne Simulator plugin has been adapted.

7. **RoboSense RS_16:** Offers link definitions to support the RoboSense RS_16 sensor. The sensor plugin has been modified based on the original RS_16 properties. Similar to the Bpearl sensor, the Velodyne Simulator plugin has been adjusted.

8. **Rover Macro:** Encompasses all link and joint definitions for the rover robot without any sensor specifications.

9. **Skid Steer Drive:** Uploads the rover model along with all three sensors and default Gazebo skid steer drive plugins.

10. **Wheel RPS Controller Drive:** A custom controller has been developed using ROS2 velocity controller to independently control all four wheels through ROS2 control.

11. **Wheel RPS Controller:** Uploads the rover model along with all three sensors and a custom ROS2 velocity controller for movement.

12. **tf description:** Uploads all sensor link and joint definitions to the robot model without any plugins for the purpose of the robot tf tree.

Feel free to explore, modify, and utilize these resources to integrate the rover model seamlessly into your ROS2 environment.