
import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import time
from functools import partial

class TFPrefixPublisher(Node):

    def __init__(self):
        super().__init__('tf_prefix_publisher')

        # publishers
        self.tf_publisher_ = self.create_publisher(CameraInfo, 'bob/oakd/rgb/preview/image_raw/camera_info', 10)
        
        self.tf_subscription = self.create_subscription(CameraInfo, 'bob/okad/rgb/preview/camera_info', self.listener_callback, 10)
        self.tf_subscription  # prevent unused variable warning

  
    def listener_callback(self, msg):
        msg_out = CameraInfo()
        msg_out = msg
        self.tf_publisher_.publish(msg_out)
        self.get_logger().info(msg_out)

    
        

def main(args=None):
    rclpy.init(args=args)

    tf_prefix_publisher = TFPrefixPublisher()

    rclpy.spin(tf_prefix_publisher)

    tf_prefix_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
