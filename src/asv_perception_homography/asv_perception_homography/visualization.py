#!/usr/bin/env python
"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>
"""

import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image, CompressedImage, Imu
from transforms3d.euler import quat2euler
from asv_perception_interfaces.msg import Homography
from asv_perception_common import utils
from asv_perception_common.NodeLazy import NodeLazy
from asv_perception_homography.calibrate_utils import create_unified_image

class HomographyVisualizationNode( NodeLazy ):

    def __init__(self):

        super().__init__('homography_visualization')
        
        self.radar_img = None
        self.rgb_img = None
        self.homography = None
        self.imu = None

        self.pub_header = None

        self.subs = []

        # publisher
        self.pub = self.advertise('image', Image, 1)

    def subscribe( self ):
        
        self.unsubscribe()

        # subscribers; approximate time sync seems to fail when restarting a rosbag; just use latest of each msg

        # rgb
        self.subs.append(self.create_subscription(CompressedImage, "rgb", self.cb_rgb, 1, buff_size=2**24))

        # radar image
        self.subs.append(self.create_subscription(Image, "radarimg", self.cb_radar, 1, buff_size=2**24))

        # homography matrix
        self.subs.append(self.create_subscription(Homography, "rgb_radarimg", self.cb_homography, 1))

        # imu
        self.subs.append(self.create_subscription(Imu, "imu", self.cb_imu, 1))

    def unsubscribe(self):

        for sub in self.subs:
            sub.unregister()
        self.subs = []

    def cb_imu( self, msg ):
        self.imu = msg
        self.publish()

    def cb_rgb( self, msg ):
        
        self.rgb_img = utils.convert_ros_msg_to_cv2( msg )
        self.pub_header = msg.header
        self.publish()
    
    def cb_radar(self, msg ):
        self.radar_img = utils.convert_ros_msg_to_cv2( msg )
        self.publish()

    def cb_homography( self, msg ):
        # convert float[9] to numpy 3x3
        #   then invert the homography, since we want radar --> rgb for image creation
        self.homography = np.linalg.inv( np.array( msg.values ).reshape((3,3)) )
        self.publish()

    def publish(self):

        # no subscribers/data, no work
        if self.pub.get_num_connections() < 1 or self.radar_img is None or self.rgb_img is None or self.homography is None:
            return

        # using rgb msg header
        assert self.pub_header is not None

        img = create_unified_image( self.rgb_img, self.radar_img, self.homography )

        # print some text on the resulting image
        print_text = lambda text, position : cv2.putText( img, text, position, cv2.FONT_HERSHEY_SIMPLEX, 1., (0,255,0), 2 )

        # convert imu orientation to euler angles, display
        if self.imu is not None:
            # imu.orientation is a normalized quaternion.  euler_from_quaternion returns radians
            rpy = euler_from_quaternion( [self.imu.orientation.x, self.imu.orientation.y, self.imu.orientation.z, self.imu.orientation.w] )
            rpy = np.degrees( rpy )
            print_text( "{}: {:.2f}".format( "Roll (deg)", rpy[0] ), ( 5, 50 ) )
            print_text( "{}: {:.2f}".format( "Pitch (deg)", rpy[1] ), ( 5, 100 ) )
            print_text( "{}: {:.2f}".format( "Yaw (deg)", rpy[2] ), ( 5, 150 ) )

        msg = utils.convert_cv2_to_ros_msg( img, 'bgr8' )
        msg.header = self.pub_header # match timestamp
        self.pub.publish( msg )

def main(args=None):
    rclpy.init(args=args)
    node = HomographyVisualizationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()