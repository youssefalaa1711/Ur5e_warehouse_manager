#!/usr/bin/env python3

# Import necessary libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        # Create a publisher that publishes Image messages to the topic 'camera_image'
        self.publisher_ = self.create_publisher(Image, 'camera_image', 10)
        # Create a timer that triggers the publish_image method every 0.01 seconds
        self.timer_ = self.create_timer(0.01, self.publish_image)
        # Log information that video publishing has started
        self.get_logger().info('video is publishing')
        # Initialize CvBridge for converting between OpenCV images and ROS Image messages
        self.bridge = CvBridge()
        # Open the default camera (index 0)
        self.cap = cv2.VideoCapture(0)  

    def publish_image(self):
        # Read a frame from the camera
        ret, frame = self.cap.read()
        # If the frame is successfully captured
        if ret:
            # Convert the frame from OpenCV's BGR format to ROS Image message format
            msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            # Publish the image message to the 'camera_image' topic
            self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
   
    main()
