#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Import String message type
from cv_bridge import CvBridge
import cv2
import pytesseract
from datetime import datetime
from my_robot_interfaces.msg import TargetCoordinates

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(Image, 'camera_image', self.image_callback, 10)
        #self.publisher = self.create_publisher(String, 'expiry_status', 10)
        self.publisher =self.create_publisher(TargetCoordinates,'topic',10)
        self.bridge = CvBridge()
        self.expiry_status_array = [0, 0, 0]
        self.index = 0

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        detected_text = pytesseract.image_to_string(cv_image)
        self.process_detected_text(detected_text)
        cv2.imshow("Camera Stream", frame)
        cv2.waitKey(1) 

    def process_detected_text(self, detected_text):
     self.get_logger().info('Detected Text: %s' % detected_text)
     specified_date = datetime.strptime("2024-04-26", "%Y-%m-%d")
     elements = detected_text.split()
     
     expired_flag = False 
     

     for element in elements:
        try:
            detected_date = datetime.strptime(element, "%Y-%m-%d")
            if detected_date < specified_date: #and not expired_flag:
                self.get_logger().info("box is expired")
                time.sleep(10)
                #for i in range(len(self.expiry_status_array)):
                self.expiry_status_array[self.index] = 1
                self.index += 1
                
                expired_flag = True 

            else:
                self.get_logger().info("box is valid")
                time.sleep(10)
                #for i in range(len(self.expiry_status_array)):
                self.expiry_status_array[self.index] = 0
                self.index += 1


        except ValueError:
            continue


        
        if(self.index > 2):
            self.publish_message()
            self.index = 0

    def publish_message(self):
        msg = TargetCoordinates()  
        msg.var1 = self.expiry_status_array[0]
        msg.var2 = self.expiry_status_array[1]
        msg.var3 = self.expiry_status_array[2]
        self.publisher.publish(msg)  
        self.print_array()

    def print_array(self):
        self.get_logger().info("Expiry Status Array: %s" % self.expiry_status_array)    

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    camera_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
