// Copyright (c) 2020 University of Massachusetts
// All rights reserved.
// This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
// Authors:  Tom Clunie <clunietp@gmail.com>

#include "ObstacleExtractionNode.h"

// #include <pluginlib/class_list_macros.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <rclcpp_components/register_node_macro.hpp>
#include "asv_perception_interfaces/msg/obstacle_array.hpp"

#include "utils.h"
#include "detail/PointCluster.h"

namespace {
    using namespace obstacle_id;

    static const std::string 
        TOPIC_NAME_INPUT = "input"
        , TOPIC_NAME_OUTPUT = "obstacles"
    ;
} // ns

ObstacleExtractionNode::ObstacleExtractionNode(const rclcpp::NodeOptions & options) : Node("obstacle_extraction", options)
{
    this->declare_parameter("cluster_size_min", (int64_t) cluster_sz_min_);
    this->declare_parameter("cluster_size_max", (int64_t) cluster_sz_max_);
    this->declare_parameter("cluster_tolerance", cluster_tolerance_);
    this->declare_parameter("cluster_area_min", cluster_area_min_);
    this->declare_parameter("cluster_area_max", cluster_area_max_);

    // Mandatory parameters
    if ( !this->get_parameter("cluster_tolerance", this->cluster_tolerance_ ))
    {
        RCLCPP_ERROR(this->get_logger(), "[%s::onInit] Need a 'cluster_tolerance' parameter to be set before continuing!", this->get_name()); 
        return;
    }

    // optional parameters
    int val = 0;  
    if ( this->get_parameter("cluster_size_max", val ) && ( val >= 0 ) )
        this->cluster_sz_max_ = (std::uint32_t)val;

    if ( this->get_parameter("cluster_size_min", val ) && ( val >= 0 ) )
        this->cluster_sz_min_ = (std::uint32_t)val;

    this->get_parameter("cluster_area_max", this->cluster_area_max_ );
    this->get_parameter("cluster_area_min", this->cluster_area_min_ );

    // publisher
    this->pub_ = create_publisher<asv_perception_interfaces::msg::ObstacleArray>(TOPIC_NAME_OUTPUT, 1 );
}

RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_id::ObstacleExtractionNode);