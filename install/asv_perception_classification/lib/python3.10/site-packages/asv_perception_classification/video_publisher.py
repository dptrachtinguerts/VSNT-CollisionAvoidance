#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisherNode(Node):
    def __init__(self):
        super().__init__('video_publisher')
        
        # parameters
        self.declare_parameter('idx', 0)
        self._index = self.get_parameter('idx').value
        self.declare_parameter('source', '/home/vicente/videos_for_yolo/VideoCamera.mp4')
        self.source_ = self.get_parameter('source').value

        self.get_logger().info('Video publisher has been started')

        self.camera_publisher_ = self.create_publisher(Image, f'n{self._index}/video_topic', 10)
        self.cap_ = cv2.VideoCapture(self.source_)  # Change to your video source
        self.timer_ = self.create_timer(1, self.timer_callback)  
        self.bridge = CvBridge()
        

    def timer_callback(self):
        ret, frame = self.cap_.read()
        print(ret)
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