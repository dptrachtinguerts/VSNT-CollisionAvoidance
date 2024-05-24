#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisherNode(Node):
    def __init__(self):
        super().__init__('video_publisher')

        #self.declare_parameter('source', 0)
        #self.get_parameter('source').value
        self.get_logger().info('Video publisher has been started')
        #self.camera_publisher_ = self.create_publisher(Image, 'video_topic', 10)
        #self.cap_ = cv2.VideoCapture('VideoCamera.avi')  # Change to your video source
        self.get_logger().info('teste sadasdasdasfasfag')
        #self.timer_ = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        #self.bridge = CvBridge()
        

    def timer_callback(self):
        ret, frame = self.cap_.read()

        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.camera_publisher_.publish(msg)
            self.get_logger().info('Publishing video frame')

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()