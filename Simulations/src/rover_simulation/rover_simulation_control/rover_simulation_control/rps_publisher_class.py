"""
Author:
- Muhammad Mursaleen @ Tampere University
- muhammadmursaleen09@gmail.com

Description:
- this class will publish rps to rover_wheels in simulation 
- this class will take input in rps by subcribing the topic
"""

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
from rover_custom_msgs.msg import CmdWheelsAngVel

class RpsPublisherController(Node):

   def __init__(self):
      """
      Constructor: 
         - initialiaze the node "rps_psublisher_controller_node"
         - add defination for Publisher for wheels rps in simulation
         - add defination for Subcriber for the input of rps of the wheel
         - create a timer that will call a wheel publisher function
      """
      super().__init__('rps_psublisher_controller_node')
      self.rps_publisher_ = self.create_publisher(
         Float64MultiArray, '/velocity_controller/commands', 10)
      self.rps_subcriber = self.create_subscription(
         CmdWheelsAngVel, '/cmd_wheels_angular_vel', 
            self.set_rps_callback ,10)
     
      # order of velocity of wheel joints is below in a array
      # left_front, left_rear, right_front, right_rear
      self.__vel = np.array([0,0,0,0], float)
      self.timer_period = 0.01  # seconds

      # Creater a timer based function that will publish the rps contineously
      self.timer = self.create_timer(self.timer_period, self.publish_rps_timer)
      self.i = 0
      self.left_rps = 0
      self.right_rps = 0

   def set_rps_callback(self, msg):
      """
      Call back function for subcriber:
         Args:  msg.ang_vel_left_motor, msg.ang_vel_right_motor
         return: None
         Output: call function for rps_publisher
         Description: it will take wheel rps and will save them and also publish
            to the wheels
      """
      self.left_rps = msg.ang_vel_left_motor
      self.right_rps =msg.ang_vel_right_motor
      self.publish_rps_timer()


   # it will publish the left & right wheel rps that has been already assigned
   def publish_rps_timer(self):

      """
      Description: Timer based publisher for rps of 4 wheels joints it will
         publish the rps to all four wheel joints for simulation
      Args: None
      Return: None
      """

      self.__vel[0] = self.left_rps  # left front
      self.__vel[1] = self.left_rps  # left back
      self.__vel[2] = self.right_rps # right front
      self.__vel[3] = self.right_rps # right back
      vel_array = Float64MultiArray(data=self.__vel) 
      self.rps_publisher_.publish(vel_array)
      std_msg = "Publishing : left_rps = "+str(self.left_rps)+" : right_rps = "+str(self.right_rps)
      self.get_logger().info(std_msg)
      self.i += 1
   
   def set_and_publish_rps(self, left_rps_, right_rps_):

      """
      Description: To set the wheel rps 
      Args: left_rps_, right_rps_
      Return: None
      """
      self.left_rps = left_rps_
      self.right_rps = right_rps_
      self.publish_rps_timer()

   def stop_rover (self):
      
      """
      Description: it will stop the robot by setting left & right wheel rps =0
      Args: none
      Return: None
      """
      self.left_rps = 0
      self.right_rps = 0
      self.publish_rps_timer()
      
