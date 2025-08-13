
import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

import time
from functools import partial

class TFPrefixPublisher(Node):

    def __init__(self):
        super().__init__('tf_prefix_publisher')

        # publishers
        self.tf_publisher_ = self.create_publisher(TFMessage, 'turtlebots/tf', 10)

        pub_qos_profile = QoSProfile(depth=5)
        pub_qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        pub_qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.static_publisher_ = self.create_publisher(TFMessage, 'turtlebots/tf_static', pub_qos_profile)

        self.publish_static_transforms()

        self.tutel_tf_prefix = "tutel/"
        self.chomik_tf_prefix = "chomik/"
        
        # TUTEL ubscribers
        self.tf_subscription = self.create_subscription(TFMessage, 'turtlebots/tutel/tf', partial(self.tf_listener_callback, tf_prefix= self.tutel_tf_prefix), 10)
        self.tf_subscription  # prevent unused variable warning

        sub_qos_profile = QoSProfile(depth=100)
        sub_qos_profile.reliability = QoSReliabilityPolicy.RELIABLE
        sub_qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        
        self.static_subscription = self.create_subscription(TFMessage, 'turtlebots/tutel/tf_static', partial(self.static_listener_callback, tf_prefix= self.tutel_tf_prefix), sub_qos_profile)
        self.static_subscription

        # CHOMIK subscribers
        self.tf_subscription = self.create_subscription(TFMessage, 'turtlebots/chomik/tf', partial(self.tf_listener_callback, tf_prefix= self.chomik_tf_prefix), 10)
        self.tf_subscription  # prevent unused variable warning
        
        self.static_subscription = self.create_subscription(TFMessage, 'turtlebots/chomik/tf_static', partial(self.static_listener_callback, tf_prefix= self.chomik_tf_prefix), sub_qos_profile)
        self.static_subscription

    def tf_listener_callback(self, msg, tf_prefix):

        msg_out = TFMessage()
        msg_out = msg

        for i in range(len(msg.transforms)):
            msg_out.transforms[i].header.frame_id = str(tf_prefix + msg.transforms[i].header.frame_id)
            msg_out.transforms[i].child_frame_id = str(tf_prefix + msg.transforms[i].child_frame_id)
            
        # tf = TransformStamped()
        # time.sleep(0.01)
        # tf.header.stamp = self.get_clock().now().to_msg()
        # tf.header.frame_id = 'tutel/odom'
        # tf.child_frame_id = 'rplidar_link'
        # tf.transform.translation.x = 0.0
        # tf.transform.translation.y = 0.0
        # tf.transform.translation.z = 0.1
        # tf.transform.rotation.x = 0.0
        # tf.transform.rotation.y = 0.0
        # tf.transform.rotation.z = 0.0
        # tf.transform.rotation.w = 0.01

        # msg_out.transforms.append(tf)

        self.get_logger().info(' In: %s' % msg.transforms[i].header.frame_id)
        self.get_logger().info('Out: %s \n \n' % msg_out.transforms[i].header.frame_id)

        self.tf_publisher_.publish(msg_out)

    def static_listener_callback(self, msg, tf_prefix):
        self.get_logger().info("recieved")
        msg_out = TFMessage()
        msg_out = msg

        for i in range(len(msg.transforms)):
            msg_out.transforms[i].header.frame_id = str(tf_prefix + msg.transforms[i].header.frame_id)
            msg_out.transforms[i].child_frame_id = str(tf_prefix + msg.transforms[i].child_frame_id)


        self.get_logger().info('Static In: %s' % msg.transforms[i].header.frame_id)
        self.get_logger().info('Static Out: %s \n \n' % msg_out.transforms[i].header.frame_id)

        self.static_publisher_.publish(msg_out)

    def publish_static_transforms(self):
        msg = TFMessage()
        tf = TransformStamped()

        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'tutel/odom'
        tf.child_frame_id = 'chomik/odom'
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = -0.5
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = 0.01

        msg.transforms.append(tf)
        self.static_publisher_.publish(msg)

        self.get_logger().info("Transform between robots published")

        

def main(args=None):
    rclpy.init(args=args)

    tf_prefix_publisher = TFPrefixPublisher()

    rclpy.spin(tf_prefix_publisher)

    tf_prefix_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
