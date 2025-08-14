
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import time


class ScanRepublisher(Node):

    def __init__(self):
        super().__init__('scan_republisher')

        # publishers
        self.tutel_publisher = self.create_publisher(LaserScan, 'turtlebots/tutel/laser_scan', 10)
        self.chomik_publisher = self.create_publisher(LaserScan, 'turtlebots/chomik/laser_scan', 10)
        

        # TUTEL ubscribers
        self.tutel_subscription = self.create_subscription(LaserScan, 'turtlebots/tutel/scan', self.tutel_listener_callback, 10)
        self.tutel_subscription  # prevent unused variable warning
        self.chomik_subscription = self.create_subscription(LaserScan, 'turtlebots/chomik/scan', self.chomik_listener_callback, 10)
        self.chomik_subscription  # prevent unused variable warning


    def tutel_listener_callback(self, msg):

        msg_out = LaserScan()
        msg_out = msg

        msg_out.header.frame_id = "tutel/rplidar_link"
        
        self.tutel_publisher.publish(msg_out)
    
    def chomik_listener_callback(self, msg):

        msg_out = LaserScan()
        msg_out = msg

        msg_out.header.frame_id = "chomik/rplidar_link"
        
        self.chomik_publisher.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)

    scan_republisher = ScanRepublisher()

    rclpy.spin(scan_republisher)

    scan_republisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
