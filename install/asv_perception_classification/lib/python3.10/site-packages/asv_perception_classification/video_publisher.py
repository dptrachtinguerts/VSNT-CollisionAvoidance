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
        
        #self.declare_parameter('source', 0)
        #self.get_parameter('source').value
        self.get_logger().info('Video publisher has been started')
        self.camera_publisher_ = self.create_publisher(Image, 'video_topic', 10)
        self.cap_ = cv2.VideoCapture('/home/vicente/VSNT-CollisionAvoidance/src/asv_perception_classification/asv_perception_classification/VideoCamera.mp4')  # Change to your video source
        #ret, frame = self.cap_.read()
        #print(frame)
        #print(ret)
        #cv2.imshow('teste', frame)
        #cv2.waitKey(10)
        self.timer_ = self.create_timer(2, self.timer_callback)  
        self.bridge = CvBridge()
        

    def timer_callback(self):
        ret, frame = self.cap_.read()
        #print(ret)
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