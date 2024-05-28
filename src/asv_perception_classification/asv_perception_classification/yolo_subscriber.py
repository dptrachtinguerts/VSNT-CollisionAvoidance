#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

from asv_perception_interfaces.msg import Classification, ClassificationArray
from asv_perception_common import utils
import time
import numpy as np

class YoloSubscriberNode(Node):
    def __init__(self):
        super().__init__('yolo_subscriber')
        self.subscription = self.create_subscription(Image,'video_topic',
                                                     self.yolo_callback, 10)
        self.bridge = CvBridge()
        self.model = YOLO('/home/vicente/VSNT-CollisionAvoidance/src/yolov8n.pt')  # Load YOLOv8 model (nano version)

        self.declare_paramete("~n_inputs", 1)
        self.n_inputs = self.get_parameter("~n_inputs").value

        # publishers array of classification array
        self.pubs = [ self.create_publisher(ClassificationArray,'~%d/output' % i, queue_size=1 ) for i in range(self.n_inputs) ]

        # image publishers for visualization/debugging
        self.pubs_img = [ self.create_publisher( Image, '~%d/image' % i, queue_size=1 ) for i in range(self.n_inputs) ]

    def yolo_callback(self, msg, idx):

        pub = self.pubs[idx]
        pub_img = self.pubs_img[idx]

        # no subscribers, no work
        if pub.get_num_connections() < 1 and pub_img.get_num_connections() < 1:
            return

        self.get_logger().info( 'Processing img with timestamp secs=%d, nsecs=%d', msg.header.stamp.secs, msg.header.stamp.nsecs )

        dets, img, annotated = self.detect(msg) # perform detection

        if pub.get_num_connections() > 0:  # publish detections
            pub.publish(dets)
        
        if pub_img.get_num_connections() > 0:  # publish annotated image
            pub_img.publish(annotated)
    
    def detect(self, image_msg):

        msg = ClassificationArray()    
        frame = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')

        results = self.model.predict(frame, 
                                # show=True,    # Show image after inference (debug)
                                save=False,     # Save prediction results
                                imgsz=320,      # Image size: changes the size of the image used in the prediction step
                                                # if smaller -> faster predictions, but probably less accuracy
                                                # if higher -> longer prediction times, and probably better accuracy
                                                # IMPORTANT: does not change the output image size, results are automatically
                                                # scaled back to the original size
                                conf=0.5,       # Confidence threshold for detection: higher means more detections, but 
                                                # also more false positives
                                classes=[8])    # Class filter: consider only the desired classes (8 == "boat")

        msg.header = time.time
        msg.image_width = results[0].orig_shape[1]
        msg.image_height = results[0].orig_shape[0]
    
        for result in results:
            cls = Classification()
            cls.label = result.boxes.cls
            cls.probability = result.boxes.conf
            roi = result.boxes.xywh # may contain multiple detections

            # darknet roi:  ( x, y, w, h ), where x and y are the centers of the detection
            #  RegionOfInterest x & y are left- and top-most coords
            cls.roi.width = np.uint32( roi[:,2] )
            cls.roi.height = np.uint32( roi[:,3] )
            cls.roi.x_offset = np.uint32( roi[:,0] - ( cls.roi.width / 2. ) )  # convert to left-most x
            cls.roi.y_offset = np.uint32( roi[:,1] - ( cls.roi.height / 2. ) ) # convert to top-most y

            msg.classifications.append(cls)
        
        return msg, frame, results[0].plot()

    def annotate( self, img, clsMsg ):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = YoloSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()