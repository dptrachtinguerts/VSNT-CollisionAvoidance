#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from asv_perception_interfaces.msg import Classification, ClassificationArray
import numpy as np

class DebugNode(Node):
    def __init__(self):
        super().__init__('classification_debug')
        self.declare_parameter("n_inputs", 1)
        self.n_inputs = self.get_parameter("n_inputs").value

        self.subs_img = [self.create_subscription(Image, 'n%d/image' % i, lambda msg, idx=i: self.callback_debug(msg, idx), 10)
                          for i in range(self.n_inputs)]
        self.bridge = CvBridge()

    def callback_debug(self, msg, idx):
        self.get_logger().info("Indice -> " + str(idx))
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow(f"annotated/{idx}", img)
        cv2.waitKey(16)

def main(args=None):
    rclpy.init(args=args)
    node = DebugNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()