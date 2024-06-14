#include "ObstacleProjectionNode.h"

#include <string>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp_components/register_node_macro.hpp>

#include "Homography.h"
#include "classified_obstacle_projection.h"

namespace {
    using namespace obstacle_id;
 
    static const std::string TOPIC_NAME_INPUT_SEGMENTATION = "segmentation";
    static const std::string TOPIC_NAME_INPUT_CLASSIFICATION = "classification";
    static const std::string TOPIC_NAME_INPUT_HOMOGRAPHY_RGB_TO_RADAR = "rgb_radar";
    static const std::string TOPIC_NAME_OUTPUT_OBSTACLES = "obstacles";
    static const std::string TOPIC_NAME_OUTPUT_CLOUD = "cloud";
}

ObstacleProjectionNode::ObstacleProjectionNode(const rclcpp::NodeOptions & options) : Node("obstacle_projection", options) {
    RCLCPP_INFO(this->get_logger(), "[ObstacleProjectionNode] %s", this->get_name());

    this->_pub_obstacle_arr = this->create_publisher<obstacle_arr_msg_t>(TOPIC_NAME_OUTPUT_OBSTACLES, 1);
    this->_pub_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(TOPIC_NAME_OUTPUT_CLOUD, 1);

    this->declare_parameter("min_height", 1.f);
    this->declare_parameter("max_height", 1.f);
    this->declare_parameter("min_depth", 1.f);
    this->declare_parameter("max_depth", 1.f);
    this->declare_parameter("resolution", 0.25f);
    this->declare_parameter("min_distance", 3.f);
    this->declare_parameter("max_distance", 100.f);
    this->declare_parameter("roi_grow_limit", 0.f);
    this->declare_parameter("roi_shrink_limit", 0.f);

    this->declare_parameter("use_segmentation", false);

    float val = 0;
    if (this->get_parameter("min_height", val) && val > 0) {
        this->_min_height = val;
    }
    if (this->get_parameter("max_height", val) && val > 0) {
        this->_max_height = val;
    }
    if (this->get_parameter("min_depth", val) && val > 0) {
        this->_min_depth = val;
    }
    if (this->get_parameter("max_depth", val) && val > 0) {
        this->_max_depth = val;
    }
    if (this->get_parameter("resolution", val) && val > 0) {
        this->_resolution = val;
    }
    if (this->get_parameter("min_distance", val) && val > 0) {
        this->_min_distance = val;
    }
    if (this->get_parameter("max_distance", val)) {
        this->_max_distance = val;
    }
    if (this->get_parameter("roi_grow_limit", val)) {
        this->_roi_grow_limit = val;
    }
    if (this->get_parameter("roi_shrink_limit", val)) {
        this->_roi_shrink_limit = val;
    }
    if (this->get_parameter("use_segmentation", val)) {
        RCLCPP_WARN(this->get_logger(), "Not implemented, do NOT set \"use_segmentation\"");
    }
}

void ObstacleProjectionNode::subscribe(void) {
    // static const std::uint32_t SYNC_QUEUE_SIZE = 10;

    std::lock_guard<std::mutex> lg(this->_mtx);

    this->_sub_rgb_radar = this->create_subscription<homography_msg_t::SharedPtr>(TOPIC_NAME_INPUT_HOMOGRAPHY_RGB_TO_RADAR, 
                                    10, std::bind(&ObstacleProjectionNode::cb_homography_rgb_radar, this, std::placeholders::_1));

    if (!this->_use_segmentation) {
        this->_sub_classification_only = this->create_subscription<classification_msg_t::SharedPtr>(TOPIC_NAME_INPUT_CLASSIFICATION,
                                                    10, std::bind(&ObstacleProjectionNode::cb_sub_classification, this, std::placeholders::_1));
    }
}

void ObstacleProjectionNode::unsubscribe(void) {
    // Apparently no official way to unsubscribe from topic

    // std::lock_guard<std::mutex> lg(this->_mtx);

    // if (!this->_use_segmentation) {
    //     this->_sub_classification_only.reset();
    // }
}

void ObstacleProjectionNode::cb_homography_rgb_radar(const homography_msg_t::SharedPtr& homogrp_msg) {
    if (!homogrp_msg) {
        RCLCPP_WARN(this->get_logger(), "Invalid homography received, ignoring");
        return;
    }
    std::lock_guard<std::mutex> lg(this->_mtx);
    this->_h_rgb_radar = homogrp_msg;
}

void ObstacleProjectionNode::cb_sub_classification(const classification_msg_t::SharedPtr& cls_msg) {
    std::lock_guard<std::mutex> lg(this->_mtx);

    if (!cls_msg || this->count_subscribers(TOPIC_NAME_OUTPUT_OBSTACLES) < 1) {
        return;
    }

    if (!this->_h_rgb_radar) {
        RCLCPP_WARN(this->get_logger(), "Homography not yet received, dropping frame");
        return;
    }

    try {
        // construct homography
        const auto 
            h_rgb_to_radar = detail::Homography( this->_h_rgb_radar->values.data() )
            ;
        
        // obstacles projected to frame
        const auto child_frame_id = this->_h_rgb_radar->child_frame_id;

        // get classified obstacles, publish obstacle message
        auto msg = asv_perception_interfaces::msg::ObstacleArray();
        auto img = cv::Mat();

        msg.obstacles = detail::classified_obstacle_projection::project( 
            img
            , *cls_msg
            , h_rgb_to_radar 
            , this->_min_height
            , this->_max_height
            , this->_min_depth
            , this->_max_depth
            , this->_min_distance
            , this->_max_distance
            , this->_roi_grow_limit
            , this->_roi_shrink_limit
            );

        // set headers
        msg.header = cls_msg->header;
        msg.header.frame_id = child_frame_id;

        for ( auto& obs : msg.obstacles ) {
            obs.header = msg.header;
        }
            
        this->_pub_obstacle_arr->publish( msg );

    } catch ( const std::exception& ex ) {
        RCLCPP_ERROR(this->get_logger(), "std::exception: %s", ex.what() );
    } catch ( ... ) {
        RCLCPP_ERROR(this->get_logger(), "unknown exception type");
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_id::ObstacleProjectionNode);