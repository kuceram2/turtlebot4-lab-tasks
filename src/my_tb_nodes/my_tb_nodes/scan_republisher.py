
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import time
from functools import partial


class ScanRepublisher(Node):

    def __init__(self):
        super().__init__('scan_republisher')

        # publishers
        self.tutel_publisher = self.create_publisher(LaserScan, 'turtlebots/tutel/laser_scan', 10)
        self.chomik_publisher = self.create_publisher(LaserScan, 'turtlebots/chomik/laser_scan', 10)
        

        # TUTEL ubscribers
        self.subscription = self.create_subscription(LaserScan, 'turtlebots/tutel/scan', partial(self.listener_callback, robot_name= "tutel"), 10)
        self.subscription  # prevent unused variable warning
        self.subscription = self.create_subscription(LaserScan, 'turtlebots/chomik/scan', partial(self.listener_callback, robot_name= "chomik"), 10)
        self.subscription  # prevent unused variable warning

        # # CHOMIK subscribers
        # self.tf_subscription = self.create_subscription(TFMessage, 'turtlebots/chomik/tf', partial(self.tf_listener_callback, tf_prefix= self.chomik_tf_prefix), 10)
        # self.tf_subscription  # prevent unused variable warning
        
        # self.static_subscription = self.create_subscription(TFMessage, 'turtlebots/chomik/tf_static', partial(self.static_listener_callback, tf_prefix= self.chomik_tf_prefix), sub_qos_profile)
        # self.static_subscription

    def listener_callback(self, msg, robot_name):

        msg_out = LaserScan()
        msg_out = msg

        msg_out.header.frame_id = robot_name + "/rplidar_link"

        if(robot_name == "tutel"): self.tutel_publisher.publish(msg_out)
        elif(robot_name == "chomik"): self.chomik_publisher.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)

    scan_republisher = ScanRepublisher()

    rclpy.spin(scan_republisher)

    scan_republisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
