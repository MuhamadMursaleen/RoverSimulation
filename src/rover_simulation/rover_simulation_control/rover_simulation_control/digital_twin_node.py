"""
Author:
- Muhammad Mursaleen @ Tampere University
- muhammadmursaleen09@gmail.com

Description:
- initialize the ros
- make a object of rover 
- this will use rps_publisher_class to publish rps to rover_wheels in simulation 
"""
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math

class DigitalTwin(Node):

   def __init__(self):
      """
      Constructor: 
         - initialiaze the node "cmd_psublisher_controller_node"
         - add defination for Publisher for wheels rps in simulation
         - add defination for Subcriber for the input of command velocity of the wheel
         - create a timer that will call a wheel publisher function
      """
      super().__init__('Digital_twin')
      self.digital_twin_publisher = self.create_publisher(Twist,
                                                 '/rover_robot/cmd_vel', 10)
      
      # Velocities from simualtion
      self.linear_velocity_x = 0
      self.angular_velocity_z =0 

      # store the previous error for pid
      self.prev_linear_error = 0
      self.prev_angular_error = 0

      # proportional multiplier
      self.linear_kp = 1
      self.angualr_kp =1

      qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,  # Set an appropriate depth value
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )

      self.camera_odom = self.create_subscription(
            Odometry,
            '/camera/pose/sample',
            self.camera_odom_callback,
            qos_profile=qos_profile
        )
      
      self.rover_odom = self.create_subscription(
            Odometry,
            '/rover_robot/odom',
            self.rover_odom_callback, 10
        )

   def rover_odom_callback(self, msg):
      """
      Call back function for Simualtion robot velocities:
         Args:  msg.command_left_motor, msg.command_right_motor
         return: None
         Description: to store simualtion velocities
      """
      self.linear_velocity_x = round(msg.twist.twist.linear.x,2)
      self.angular_velocity_z = round(msg.twist.twist.angular.z,2)

   def Cal_vel_using_pid(self,msg):
      """
      To Calcualte the error from real time to simualtion and adjust it by
         using pid.
         Args:  msg.command_left_motor, msg.command_right_motor
         return: linear and angular velocities
         Description: it will take wheel command velocity and convert into the rps
            and will save them
      """

      # Get Current velocities
      linear_velocity_x = round(msg.twist.twist.linear.x,2)
      angular_velocity_z = round(msg.twist.twist.angular.z,2)

      # Calcualte the error
      linear_error_x = linear_velocity_x - self.linear_velocity_x
      angular_error_z = angular_velocity_z - self.angular_velocity_z 

      # store sum of error
      self.prev_linear_error += linear_error_x
      self.prev_angular_error += angular_error_z

      # Calcualt proportional 
      current_linear_velocity = self.linear_kp *(linear_velocity_x + linear_error_x )
      current_angular_velocity = self.angualr_kp*(angular_velocity_z + angular_error_z)

      # to negotiate nan eror
      if (current_linear_velocity=='nan' or current_angular_velocity=='nan' ):
         current_linear_velocity =0.0
         current_angular_velocity =0.0

      # Publish the velocities
      to_send = Twist()
      to_send.linear.x= current_linear_velocity
      to_send.angular.z =current_angular_velocity
      self.digital_twin_publisher.publish(to_send)

   def camera_odom_callback(self, msg):

      """
      Call back function for real robot velocities:
         Args:  msg.command_left_motor, msg.command_right_motor
         return: None
         Output: call function for rps_publisher
         Description: it will take wheel command velocity and convert into the rps
            and will save them and also publish to the wheels
      """
      
      linear_velocity_x = round(msg.twist.twist.linear.x,2)
      angular_velocity_z = round(msg.twist.twist.angular.z,2)
   
      current_linear_velocity = linear_velocity_x
      current_angular_velocity = angular_velocity_z

      # to negotiate nan eror
      if (current_linear_velocity=='nan' or current_angular_velocity=='nan' ):
         current_linear_velocity =0.0
         current_angular_velocity =0.0

      
      to_send = Twist()
      to_send.linear.x  = current_linear_velocity
      to_send.angular.z = current_angular_velocity
      self.digital_twin_publisher.publish(to_send)


   def YAW(self,msg):
      """
      Yaw:
         Args:  msg
         return: yaw (in degrees)
         Description: it will convert quaternions angles to yaw
      """
      
      q0 = msg.pose.pose.orientation.w
      q1 = msg.pose.pose.orientation.x
      q2 = msg.pose.pose.orientation.y
      q3 = msg.pose.pose.orientation.z
      yaw = round(math.degrees(math.atan2(2.0*(q0*q3 + q1*q2),
                                                (1.0-2.0*(q2*q2 + q3*q3)))),2)
      return yaw


   # it will stop the robot by setting left & right wheel rps =0
   def stop_rover (self):
      """
      Description: it will stop the robot by setting angular & linear velocity =0
      Args: none
      Return: None
      """
      to_send = Twist()
      to_send.linear.x= 0
      to_send.angular.z =0
      self.digital_twin_publisher.publish(to_send)



def main(args=None):
   try:
      # Initialize the ros
      rclpy.init()

      # make a object of rover for publishing rps to rover_wheels in simulation 
      digital_twin_rover = DigitalTwin()
      rclpy.spin(digital_twin_rover)

   except KeyboardInterrupt:

      # Stop the rover
      digital_twin_rover.stop_rover()
      
      # destroy the node
      digital_twin_rover.destroy_node()

      # shutdown the ros
      rclpy.shutdown()


if __name__ == '__main__':
   main()