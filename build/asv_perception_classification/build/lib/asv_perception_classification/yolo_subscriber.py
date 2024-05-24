#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloSubscriberNode(Node):
    def __init__(self):
        super().__init__('yolo_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'video_topic',
            self.yolo_callback,
            10)
        self.bridge = CvBridge()
        self.model = YOLO('/home/vicente/VSNT-CollisionAvoidance/src/yolov8n.pt')  # Load YOLOv8 model (nano version)

    def yolo_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame)  # Perform detection
        annotated_frame = results[0].plot()  # Render results
        cv2.imshow('YOLOv8 Detection', annotated_frame)
        cv2.waitKey(1)
        self.get_logger().info('Processed video frame with YOLOv8')

def main(args=None):
    rclpy.init(args=args)
    node = YoloSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()