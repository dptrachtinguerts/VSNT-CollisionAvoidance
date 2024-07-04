// Copyright (c) 2020 University of Massachusetts
// All rights reserved.
// This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
// Authors:  Tom Clunie <clunietp@gmail.com>

#include "PointCloudConcatNode.h"

#include <rclcpp_components/register_node_macro.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace {
    using namespace obstacle_id;

    static const std::string 
        TOPIC_NAME_INPUT = "input"
        , TOPIC_NAME_OUTPUT_FULL = "full"
        , TOPIC_NAME_OUTPUT_CURRENT = "current"
    ;
} // ns

PointCloudConcatNode::PointCloudConcatNode(const rclcpp::NodeOptions & options) : Node("point_cloud_concat", options)
{
    // parameters
    this->declare_parameter("decay_time", this->decay_time_);
    this->get_parameter("decay_time", this->decay_time_);

    assert( this->decay_time_ > 0. );

    // publishers
    this->pub_full_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(TOPIC_NAME_OUTPUT_FULL, 1);
    this->pub_current_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(TOPIC_NAME_OUTPUT_CURRENT, 1);
}

void PointCloudConcatNode::subscribe()
{
    lock_type_ lg( this->mtx_ );

    this->last_full_publish_= now();

    this->sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2::SharedPtr>(
        TOPIC_NAME_INPUT, 1, std::bind(&PointCloudConcatNode::sub_callback, this, std::placeholders::_1));
}

void PointCloudConcatNode::unsubscribe ()
{
    ;
}

void PointCloudConcatNode::sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud)
{
    lock_type_ lg( this->mtx_ );

    rclcpp::Time curr_time = now();

    // cleanup old segments
    std::vector<std::pair<sensor_msgs::msg::PointCloud2::SharedPtr, rclcpp::Time>> valid = {};

    for (auto& seg : this->segments_) {
        
        const auto diff = std::abs((curr_time - seg.second).seconds()); // using abs in case of time jumps due to rosbag restart

        /* Descarta o segmento de PC se a diferença de tempo 
        for maior que decay_time */
        if ( diff <= this->decay_time_ ) {
            valid.emplace_back(std::move(seg));
        }
    }

    valid.emplace_back( cloud, curr_time ); // use msg header time?
    this->segments_ = std::move( valid );

    const bool time_for_full_publish = ( ( curr_time - this->last_full_publish_ ).seconds() > this->decay_time_ );

    // concat and publish when we have a subscriber to 'current', or when we're due for a full publish
    if ( ( this->count_subscribers(TOPIC_NAME_OUTPUT_CURRENT) < 1 ) && !time_for_full_publish )
        return;

    try {
        /*
        https://github.com/ros-perception/perception_pcl/blob/b1917efe78112300473590f25e93fd8edbdac7c4/pcl_conversions/include/pcl_conversions/pcl_conversions.h#L611
        pcl_conversions:  bool concatenatePointCloud (const sensor_msgs::PointCloud2 &cloud1,
                                const sensor_msgs::PointCloud2 &cloud2,
                                sensor_msgs::PointCloud2 &cloud_out)
        */

        auto result = sensor_msgs::msg::PointCloud2();
        for ( auto& pc_pair : this->segments_ ) {
            auto& pc = pc_pair.first;
            /* Provavelmente errado: pc == nullptr && pc->width == 0 
                - Pode ser uma validação para garantir que o erro é a operação
                de concatenação, e não o ponteiro para PC ou o comprimento da 
                lista */
            if ( pc != nullptr && ( pc->width > 0 ) && !pcl::concatenatePointCloud( result, *pc, result )) {
                RCLCPP_ERROR(this->get_logger(), "Error concatenating point clouds");
            }
        }
            
        // use latest header
        result.header = cloud->header;

        if ( this->count_subscribers(TOPIC_NAME_OUTPUT_CURRENT) > 0 ) {
            this->pub_current_->publish( result );
        }

        if ( time_for_full_publish && ( this->count_subscribers(TOPIC_NAME_OUTPUT_FULL) > 0 ) ) {
            this->pub_full_->publish( result );
            this->last_full_publish_=curr_time;
        }

    } catch ( const std::exception& ex ) {  // pcl exceptions inherit from std::runtime_error
        RCLCPP_ERROR(this->get_logger(), "std::exception: %s", ex.what() );
    } catch ( ... ) {
        RCLCPP_ERROR(this->get_logger(), "unknown exception type");
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_id::PointCloudConcatNode);