// Copyright (c) 2020 University of Massachusetts
// All rights reserved.
// This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
// Authors:  Tom Clunie <clunietp@gmail.com>

#include "ObstacleProjectionNodelet.h"

#include <pluginlib/class_list_macros.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <asv_perception_interfaces/msg/obstacle_array.hpp>
#include <cv_bridge/cv_bridge.h>

#include "asv_perception_obstacleid/classified_obstacle_projection.h"
#include "asv_perception_obstacleid/obstacle_projection.h"

#include <rclcpp_components/register_node_macro.hpp>
#include "asv_perception_interfaces/msg/obstacle_array.hpp"

namespace {
    using namespace obstacle_id;

    static const std::string 
        TOPIC_NAME_INPUT_SEGMENTATION = "segmentation", TOPIC_NAME_INPUT_CLASSIFICATION = "classification", 
        TOPIC_NAME_INPUT_HOMOGRAPHY_RGB_TO_RADAR = "rgb_radar", TOPIC_NAME_OUTPUT_OBSTACLES = "obstacles", 
        TOPIC_NAME_OUTPUT_CLOUD = "cloud";

} // ns


//////////////////////////////////////////////////////////////////////////////////////////////
ObstacleProjectionNodelet::ObstacleProjectionNodelet(const rclcpp::NodeOptions & options) : Node("obstacle_projection", options)
{
    // NODELET_DEBUG("[%s::onInit] Initializing node", getName().c_str() );
    RCLCPP_INFO(this->get_logger(), "[%s::onInit] Initializing node", this->get_name());

    // advertise publishers
    _pub = this->create_publisher<asv_perception_interfaces::msg::ObstacleArray>( TOPIC_NAME_OUTPUT_OBSTACLES, 1 );
    _pub_cloud = this->create_publisher<sensor_msgs::msg::PointCloud2>(TOPIC_NAME_OUTPUT_CLOUD, 1);

    // get parameters
    float val = 0; 
    this->declare_parameter("min_height", val);
    this->declare_parameter("max_height", val); 
    this->declare_parameter("min_depth", val); 
    this->declare_parameter("max_depth", val); 
    this->declare_parameter("resolutiion", val); 
    this->declare_parameter("min_distance", val);
    this->declare_parameter("max_distance", val);  

    if (val = this->get_parameter("min_height").as_double() > 0)
        this->_min_height = val;

    if (val = this->get_parameter("max_height").as_double() > 0)
        this->_max_height = val;

    if (val = this->get_parameter("min_depth").as_double() > 0)
        this->_min_depth = val;

    if (val = this->get_parameter("max_depth").as_double() > 0 )
        this->_max_depth = val;

    if (val = this->get_parameter("resolution").as_double() > 0)
        this->_resolution = val;

    if (val = this->get_parameter("min_distance").as_double() > 0)
        this->_min_distance = val;

    if (val = this->get_parameter("max_distance").as_double() > 0)
        this->_max_distance = val;

    this->declare_parameter("roi_grow_limit", 0);  
    this->declare_parameter("roi_shrink_limit", 0);  
    this->declare_parameter("use_segmentation", 0);  

    this->_roi_grow_limit = this->get_parameter("roi_grow_limit").get_value();
    this->_roi_shrink_limit = this->get_parameter("roi_shrink_limit").get_value();
    this->_use_segmentation = this->get_parameter("use_segmentation").get_value();

    //onInitPostProcess ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleProjectionNodelet::subscribe ()
{
    static const std::uint32_t SYNC_QUEUE_SIZE = 10;

    std::lock_guard<std::mutex> lg( this->_mtx );

    this->_sub_rgb_radar = this->create_subscription<homography_msg_type>( 
        TOPIC_NAME_INPUT_HOMOGRAPHY_RGB_TO_RADAR, 
        1, 
        std::bind(&ObstacleProjectionNodelet::cb_homography_rgb_radar, this, _1 )
    );

    // if ( this->_use_segmentation ) { 
    
    //     this->_sub_segmentation.subscribe(  TOPIC_NAME_INPUT_SEGMENTATION, 1 );
    //     this->_sub_classification.subscribe( TOPIC_NAME_INPUT_CLASSIFICATION, 1 );

    //     this->_seg_cls_sync.reset(new _seg_cls_synchronizer_type(_seg_cls_sync_policy_type( SYNC_QUEUE_SIZE ),
    //         _sub_segmentation, _sub_classification));

    //     this->_seg_cls_sync->registerCallback( bind (&ObstacleProjectionNodelet::sub_callback, this, _1, _2 ) );
    //}
    //else {  // classification only

    this->_sub_classification_only = this->create_subscription<classification_msg_type> (
        TOPIC_NAME_INPUT_CLASSIFICATION, 1, std::bind( &ObstacleProjectionNodelet::sub_callback, this, _1 ));

    //}
}

//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleProjectionNodelet::unsubscribe ()
{
    std::lock_guard<std::mutex> lg( this->_mtx );

    // if ( this->_use_segmentation ) {
    //     this->_sub_segmentation.unsubscribe();
    //     this->_sub_classification.unsubscribe();
    //} else {
    this->_sub_classification_only.shutdown();
    //}

    this->_sub_rgb_radar.shutdown();
}

void ObstacleProjectionNodelet::cb_homography_rgb_radar( const typename homography_msg_type::SharedPtr& h ) {

    if ( !h ) {
        //NODELET_WARN( "Invalid homography received, ignoring" );
        RCLCPP_WARN(this->get_logger(), "Invalid homography received, ignoring");
        return;
    }
    std::lock_guard<std::mutex> lg( this->_mtx );
    this->_h_rgb_radar = h;
}

// segmentation + classification msg
// void ObstacleProjectionNodelet::sub_callback ( 
//     const typename segmentation_msg_type::SharedPtr& seg_msg, 
//     const typename classification_msg_type::SharedPtr& cls_msg)
// {

//     std::lock_guard<std::mutex> lg( this->_mtx );
    
//     // No subscribers/data, no work
//     if (!seg_msg || !cls_msg || (( this->_pub.getNumSubscribers () < 1 ) 
//             && ( this->_pub_cloud.getNumSubscribers() < 1 )))
//         return;

//     // check we have homography, warn
//     if ( !this->_h_rgb_radar ) {
//         // NODELET_WARN( "Homography not yet received, dropping frame" );
//         RCLCPP_WARN(this->get_logger(), "Homography not yet received, dropping frame");
//         return;
//     }

//     // we have:
//     //  seg img, classifications, homography
//     auto img_ptr = cv_bridge::toCvCopy( seg_msg, sensor_msgs::image_encodings::MONO8 );
    
//     if ( !img_ptr || img_ptr->image.empty() ) {
//         // NODELET_WARN( "Invalid segmentation image, dropping frame" );
//         RCLCPP_WARN(this->get_logger(), "Invalid segmentation image, dropping frame");
//         return;
//     }

//     // get cv:Mat reference
//     auto& img = img_ptr->image;

//     if (( img.cols != cls_msg->image_width ) || ( img.rows != cls_msg->image_height)) 
//     {
//         // NODELET_WARN( "Segmentation/classification shape mismatch, dropping frame" );
//         RCLCPP_WARN(this->get_logger(), "Segmentation/classification shape mismatch, dropping frame");
//         return;
//     }

//     try {

//         // construct homography
//         const auto 
//             h_rgb_to_radar = asv_perception_obstacleid::Homography( this->_h_rgb_radar->values.data() );
//         // obstacles projected to frame
//         const auto child_frame_id = this->_h_rgb_radar->child_frame_id;

//         // get classified obstacles, publish obstacle message
//         auto msg = asv_perception_interfaces::ObstacleArray();
        
//         msg.obstacles = detail::classified_obstacle_projection::project( 
//             img,
//             *cls_msg,
//             h_rgb_to_radar ,
//             this->_min_height,
//             this->_max_height,
//             this->_min_depth,
//             this->_max_depth,
//             this->_min_distance,
//             this->_max_distance,
//             this->_roi_grow_limit,
//             this->_roi_shrink_limit
//             );

//         // set headers
//         msg.header = cls_msg->header;
//         msg.header.frame_id = child_frame_id;

//         for ( auto& obs : msg.obstacles ) {
//             obs.header = msg.header;
//         }
            
//         this->_pub.publish( msg );

//         // debug img
//         /*
//         if ( this->_pub_debug_img.getNumSubscribers() > 0 ) {
//             auto debug_img_msg = sensor_msgs::Image();
//             cv_bridge::CvImage debug_img;
//             debug_img.encoding = sensor_msgs::image_encodings::MONO8;
//             debug_img.image = img;
//             debug_img.toImageMsg( debug_img_msg );
//             debug_img_msg.header = cls_msg->header;
//             this->_pub_debug_img.publish( debug_img_msg );
//         }
//         */
        
//         // unclass pointcloud
//         if ( this->_pub_cloud.getNumSubscribers() > 0 ) {

//             auto cloud = detail::obstacle_projection::project( 
//                 img,
//                 h_rgb_to_radar,
//                 this->_max_height,
//                 this->_max_depth,
//                 this->_resolution,
//                 this->_max_distance
//                 );

//             sensor_msgs::PointCloud2::Ptr output_blob( new sensor_msgs::PointCloud2() );

//             pcl::toROSMsg ( cloud, *output_blob );
//             output_blob->header = cls_msg->header;
//             output_blob->header.frame_id = child_frame_id;

//             // publish
//             this->_pub_cloud.publish( output_blob );
//         }
//   } catch ( const std::exception& ex ) {
//     // ROS_ERROR("std::exception: %s", ex.what() );
//     RCLCPP_ERROR(this->get_logger(), "std::exception: %s", ex.what());
//   } catch ( ... ) {
//     // ROS_ERROR("unknown exception type");
//     RCLCPP_ERROR(this->get_logger(), "unknown exception type");
//   }
    
// }

// classification message only
void ObstacleProjectionNodelet::sub_callback ( 
    const typename classification_msg_type::SharedPtr& cls_msg)
{

    std::lock_guard<std::mutex> lg( this->_mtx );
    
    // No subscribers/data, no work
    if ( !cls_msg || this->count_subscribers(TOPIC_NAME_OUTPUT_OBSTACLES) < 1 )
        return;
        
    // check we have homography, warn
    if ( !this->_h_rgb_radar ) {
        // NODELET_WARN( "Homography not yet received, dropping frame" );
        RCLCPP_WARN(this->get_logger(), "Homography not yet received, dropping frame");
        return;
    }
    
    try {

        // construct homography
        const auto
            h_rgb_to_radar = detail::Homography( this->_h_rgb_radar->values.data() );
        
        // obstacles projected to frame
        const auto child_frame_id = this->_h_rgb_radar->child_frame_id;

        // get classified obstacles, publish obstacle message
        auto msg = asv_perception_interfaces::ObstacleArray();
        auto img = cv::Mat();

        msg.obstacles = detail::classified_obstacle_projection::project( 
            img,
            *cls_msg,
            h_rgb_to_radar ,
            this->_min_height,
            this->_max_height,
            this->_min_depth,
            this->_max_depth,
            this->_min_distance,
            this->_max_distance,
            this->_roi_grow_limit,
            this->_roi_shrink_limit
            );

        // set headers
        msg.header = cls_msg->header;
        msg.header.frame_id = child_frame_id;

        for ( auto& obs : msg.obstacles ) {
            obs.header = msg.header;
        }
            
        this->_pub.publish( msg );
    

  } catch ( const std::exception& ex ) {
    // ROS_ERROR("std::exception: %s", ex.what() );
    RCLCPP_ERROR(this->get_logger(), "std::exception: %s", ex.what());

  } catch ( ... ) {
    // ROS_ERROR("unknown exception type");
    RCLCPP_ERROR(this->get_logger(), "unknown exception type");
  }
    
}

//PLUGINLIB_EXPORT_CLASS(obstacle_id::ObstacleProjectionNodelet, nodelet::Nodelet)
//RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_id::ObstacleProjectionNodelet)