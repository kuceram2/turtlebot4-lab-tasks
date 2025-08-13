import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import sys

class OrangeBlobDetector(Node):
    def __init__(self):
        super().__init__('orange_blob_detector')
        self.declare_parameter('namespace', '')
        namespace = self.get_parameter('namespace').get_parameter_value().string_value
        
        #namespace = sys.argv[1]  # <-- uncomment this if you want to run the file as raw python script 
        self.subscription = self.create_subscription(
            Image,
            f'/{namespace}/oakd/rgb/preview/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Twist, f'/{namespace}/cmd_vel', 10)
        
        self.target_x = 0
        self.target_y = 0
        

    def listener_callback(self, msg):
        self.get_target(msg)
        msg_out = Twist()
        # If target is detected
        if self.target_x == -1 or self.target_y == -1:
            msg_out.linear.x = 0.0
            msg_out.angular.z = 0.2
        else:
            if self.target_x > 200 and self.target_x < 280: msg_out.linear.x = 0.5
            elif self.target_x < 200: msg_out.angular.z = 0.3
            elif self.target_x >280: msg_out.angular.z = -0.3
        
        self.publisher.publish(msg_out)
            


    def get_target(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame = cv2.resize(frame, (480, 480))

        

        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # orange color range in HSV
        lower_orange = np.array([11,148,153])
        upper_orange = np.array([25, 255, 255])
        
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # remove noise
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        biggest_area = 5
        target_x = 0
        target_y = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 5:  # noise filter
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 1)
            if area > biggest_area:
                biggest_area = area
                target_x = int(x + (w/2))
                target_y = int(y + (h/2))

        if len(contours) == 0:
            target_x = -1
            target_x = -1
            
        self.target_x = target_x
        self.target_y = target_y                
        cv2.circle(frame, center=(target_x, target_y), radius=5, color=(0,255,0), thickness=6)
        
        cv2.imshow("Camera Feed with Blobs", frame)
        self.get_logger().info(f"Target X: {self.target_x} Y: {self.target_y}")
        #cv2.imshow("Mask", mask)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = OrangeBlobDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
