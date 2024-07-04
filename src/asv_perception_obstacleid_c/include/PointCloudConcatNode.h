// Copyright (c) 2020 University of Massachusetts
// All rights reserved.
// This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
// Authors:  Tom Clunie <clunietp@gmail.com>

#ifndef POINTCLOUDCONCATNODELET_H
#define POINTCLOUDCONCATNODELET_H

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace obstacle_id
{
    /*
        Concatenates multiple partial pointcloud segments received over time into a single pointcloud

        Subscriptions:
            ~input:     [sensor_msgs/PointCloud2] input pointcloud segment

        Publications:
            ~full:      [sensor_msgs/PointCloud2]  pointcloud representing 1 complete pointcloud after having received all N segments
            ~current:   [sensor_msgs/PointCloud2]  pointcloud representing most current information, published after receiving each segment

        Parameters:
            ~decay_time:   [int, required]  number of seconds to store a partial pointcloud.  A full pointcloud is published after each decay_time interval

    */
    class PointCloudConcatNode : public rclcpp::Node {
        public:        
            // constructor
            PointCloudConcatNode(const rclcpp::NodeOptions & options);
                                            
        protected:

            /** \brief LazyNodelet connection routine. */
            void subscribe();
            void unsubscribe();

            // the callback function to handle input from subscription
            void sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr&);
        
        private:

            rclcpp::Subscription<sensor_msgs::msg::PointCloud2::SharedPtr>::SharedPtr sub_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_full_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_;

            std::mutex mtx_;
            using lock_type_ = std::lock_guard<std::mutex>;

            // segments, time storage      
            std::vector<std::pair<sensor_msgs::msg::PointCloud2::SharedPtr, rclcpp::Time>> segments_;

            // segment survival time (>0)
            float decay_time_ = 0.f;

            // last full publish time
            rclcpp::Time last_full_publish_ = now();
    };
}

#endif  //#ifndef 