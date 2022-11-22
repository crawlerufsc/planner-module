import sys
import rclpy
from rclpy.node import Node
from crawler_ros_api_interface.src.srv import CrawlerRequestCommand

def main ():
    rclpy.init()
    rclpy.shutdown()

if __name__ == '__main__':
    main()