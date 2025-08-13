import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import sys
import time
class OrangeBlobDetector(Node):
    def __init__(self):
        super().__init__('orange_blob_detector')
        namespace = sys.argv[1]
        self.subscription = self.create_subscription(
            Image,
            f'/{namespace}/oakd/rgb/preview/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Twist, f'/{namespace}/cmd_vel', 10)
        
        self.target_x = 0
        self.target_y = 0
        self.res = 247
        self.detection_number = 0
        self.detector = cv2.QRCodeDetector()
        

    def listener_callback(self, msg):
        self.get_target(msg)
        # msg_out = Twist()
        # # If target is detected
        # if self.target_x == -1 or self.target_y == -1:
        #     msg_out.linear.x = 0.0
        #     msg_out.angular.z = 0.2
        # else:
        #     if self.target_x > 200 and self.target_x < 280: msg_out.linear.x = 0.5
        #     elif self.target_x < 200: msg_out.angular.z = 0.5
        #     elif self.target_x >280: msg_out.angular.z = -0.5
        
        # self.publisher.publish(msg_out)
            


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


        data, bbox, _ = self.detector.detectAndDecode(frame)

        if bbox is not None:
            # Draw bounding box
            for i in range(len(bbox)):
                #cv2.line(frame, (100, 100), (200, 200), (255, 0, 0), 2)
                cv2.circle(frame, (int(bbox[0][0][0]), int(bbox[0][0][1])), 5, (0,255,0), 5)
                cv2.circle(frame, (int(bbox[0][2][0]), int(bbox[0][2][1])), 5, (255,0,0), 5)

                pts = np.array(bbox[0], np.int32)

                cv2.polylines(frame, [pts], True, (0,255,0), 5 )
                #print(bbox)
                pass
            if data:
                print(f"Decoded Data: {data} | Count: {self.detection_number}")
                self.detection_number += 1
                cv2.putText(frame, data, (int(bbox[0][0][0]), int(bbox[0][0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        cv2.imshow("Camera Feed with Blobs", frame)
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
