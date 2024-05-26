#!/usr/bin/env python

"""
Copyright (c) 2020 University of Massachusetts
All rights reserved.
This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
Authors:  Tom Clunie <clunietp@gmail.com>

"""

import rclpy
import numpy as np
from std_msgs.msg import Header, Empty
from sensor_msgs.msg import Imu

from tf.transformations import euler_from_quaternion
from asv_perception_interfaces.msg import Homography
from asv_perception_common import utils

from scripts_homography.calibrate_utils import create_warp_matrix, get_radar_to_world_matrix
from scripts_homography.FeedforwardImuController import FeedforwardImuController

class HomographyNode():

    def __init__(self):
        super().__init__('homography_node')

        # feed forward imu controller
        self.ctrl = FeedforwardImuController()
        self.last_imu_msg = None

        self.has_published = False

        # publish with latch in case of no IMU/testing/etc
        self.pub_rgb_radarimg_ = self.create_publisher(Homography, '~rgb_radarimg', 1)
        self.pub_radarimg_radar_ = self.create_publisher(Homography, '~radarimg_radar', 1)
        self.pub_rgb_radar_ = self.create_publisher(Homography, '~rgb_radar', 1)

        # subscriptions:

        # imu
        self.sub_imu = self.create_subscription(Imu,'~imu', self.cb_imu, 1)

        # refresh notification from calibration tool; useful for visualization updates when rosbag is paused/stopped
        self.sub_refresh = self.create_subscription(Empty,'~refresh', self.cb_refresh, 1)
        
    def cb_refresh(self, msg):
        self.publish()

    def cb_imu(self, msg):
        self.last_imu_msg = msg
        self.publish()

    # publish from rgb to other frame
    def publish_homography(self, pub, M, t, frame_id, child_frame_id):
        msg = Homography()
        msg.header.stamp = t
        msg.header.frame_id = frame_id
        msg.child_frame_id = child_frame_id
        msg.values = np.ravel(M)
        pub.publish(msg)
        self.has_published = True

    def publish(self):

        ### VERIFICAR SE É NECESSÁRIO

        # check for early exit
        if self.has_published and self.pub_rgb_radarimg.get_num_connections() < 1 and self.pub_radarimg_radar.get_num_connections() < 1 and self.pub_rgb_radar.get_num_connections() < 1:
            return

        # message time
        t = self.get_clock().now().to_msg()

        # update ff ctrl params, update
        self.ctrl.yaw_alpha = self.get_parameter('~imu_yaw_alpha').value
        self.ctrl.yaw_beta = self.get_parameter('~imu_yaw_beta').value
        self.ctrl.yaw_gamma = self.get_parameter('~imu_yaw_gamma').value
        
        #self.ctrl.pitch_alpha = self.get_parameter('~imu_pitch_alpha').get_parameter_value().double_value
        #self.ctrl.yaw_alpha = rospy.get_param('~imu_yaw_alpha', 0. )
        #self.ctrl.yaw_beta = rospy.get_param('~imu_yaw_beta', 0. )
        #self.ctrl.yaw_gamma = rospy.get_param('~imu_yaw_gamma', 0. )

        self.ctrl.pitch_alpha = self.get_parameter('~imu_pitch_alpha').value
        self.ctrl.pitch_beta = self.get_parameter('~imu_pitch_beta').value 
        self.ctrl.pitch_gamma = self.get_parameter('~imu_pitch_gamma').value

        self.ctrl.roll_alpha = self.get_parameter('~imu_roll_alpha').value 
        self.ctrl.roll_beta =  self.get_parameter('~imu_roll_beta').value
        self.ctrl.roll_gamma = self.get_parameter('~imu_roll_gamma').value 

        if not self.last_imu_msg is None:
            self.ctrl.update(self.last_imu_msg)

        # rgb to radar
        #  create_warp_matrix computes radar to rgb, we want the inverse
        M_rgb_radar = np.linalg.inv(create_warp_matrix(self.get_parameter('~radar_img_w').value, 
                                                       self.get_parameter('~radar_img_w').value, 
                                                       self.get_parameter('~yaw').value - np.degrees(self.ctrl.yaw), 
                                                       self.get_parameter('~pitch').value - np.degrees(self.ctrl.pitch), 
                                                       self.get_parameter('~roll').value - np.degrees(self.ctrl.roll), 
                                                       1., 
                                                       self.get_parameter('~fovy').value, 
                                                       self.get_parameter('~tx').value,
                                                       self.get_parameter('~ty').value, 
                                                       self.get_parameter('~tz').value ))

        rgb_frame_id = self.get_parameter("~rgb_frame_id").value
        radarimg_frame_id = self.get_parameter("~radarimg_frame_id").value
        radar_frame_id = self.get_parameter("~radar_frame_id").value

        self.publish_homography(self.pub_rgb_radarimg_, M_rgb_radar, t, rgb_frame_id, radarimg_frame_id)

        # radar to robot
        #  multiply radar range by 2 to get diameter
        M_radar_robot = get_radar_to_world_matrix(self.get_parameter('~radar_img_w').value, 
                                                  2. * self.get_parameter('~radar_range').value)
        
        self.publish_homography(self.pub_radarimg_radar_, M_radar_robot, t, radarimg_frame_id, radar_frame_id)

        # rgb to robot is (radar_to_robot)*(rgb_to_radar)
        M_rgb_robot = np.matmul(M_radar_robot, M_rgb_radar)
        self.publish_homography( self.pub_rgb_radar_, M_rgb_robot, t, rgb_frame_id, radar_frame_id )
        

def main(args=None):
    rclpy.init(args=args)
    node = HomographyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()