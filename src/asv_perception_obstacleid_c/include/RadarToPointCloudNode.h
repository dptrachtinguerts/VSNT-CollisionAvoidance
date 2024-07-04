// Copyright (c) 2020 University of Massachusetts
// All rights reserved.
// This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
// Authors:  Tom Clunie <clunietp@gmail.com>

#ifndef RADARTOPOINTCLOUDNODELET_H
#define RADARTOPOINTCLOUDNODELET_H

#include <mutex>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <asv_perception_interfaces/msg/radar_segment.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "defs.h"

namespace obstacle_id
{
    /*
        Description
            Converts RadarSegment messages to pointcloud
            
        Subscriptions:
            ~input:     [asv_perception_common/RadarSegment] input RadarSegment.  RadarSpoke angles are in degrees.

        Publications:
            ~segment:       [sensor_msgs/PointCloud2] the most current RadarSegment converted to a pointcloud
            ~full:          [sensor_msgs/PointCloud2] The full radar scan, published after a complete sweep
            ~full_current:  [sensor_msgs/PointCloud2] The most current, full radar scan, published after receiving the most recent RadarSegment

        Parameters:
            ~min_intensity:    [uint8, default=1]  minimum return intensity before publishing a point
            ~angle_offset:     [float, default=0.0]  angle offset (added to RadarSpoke angle) to correct for orientation forward
            ~frame_id:         [string]  output header frame id.  By default, the header is copied from the source RadarSegment
    */
    class RadarToPointCloudNode : public rclcpp::Node
    {
        public:
            RadarToPointCloudNode(const rclcpp::NodeOptions & options);
                                        
        protected:

            /** \brief LazyNodelet connection routine. */
            void subscribe();
            void unsubscribe();

            // the callback function to handle input from subscription
            void sub_callback ( const asv_perception_interfaces::msg::RadarSegment::SharedPtr& );
            
        private:

            std::mutex mtx_;
            using lock_type_ = std::lock_guard<std::mutex>;

            // segments storage
            std::deque<std::pair<sensor_msgs::msg::PointCloud2, float>> segments_;

            // parameters
            float angle_offset_ = 0.f;
            std::uint8_t min_intensity_ = (std::uint8_t)1;
            std::string frame_id_;

            rclcpp::Subscription<asv_perception_interfaces::msg::RadarSegment::SharedPtr>::SharedPtr sub_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_segment_; 
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_full_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_full_current_;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif  //#ifndef 