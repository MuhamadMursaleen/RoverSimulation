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
from .rps_publisher_class import RpsPublisherController

def main(args=None):

   try:
      # Initialize the ros
      rclpy.init()

      # make a object of rover for publishing rps to rover_wheels in simulation 
      publish_rps = RpsPublisherController()
      rclpy.spin(publish_rps)

   except KeyboardInterrupt:

      # Stop the rover
      publish_rps.stop_rover()
      
      # destroy the node
      publish_rps.destroy_node()

      # shutdown the ros
      rclpy.shutdown()


if __name__ == '__main__':
   main()