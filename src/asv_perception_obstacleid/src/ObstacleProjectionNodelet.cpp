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

namespace {
    using namespace obstacle_id;

    static const std::string 
        TOPIC_NAME_INPUT_SEGMENTATION = "segmentation", TOPIC_NAME_INPUT_CLASSIFICATION = "classification", 
        TOPIC_NAME_INPUT_HOMOGRAPHY_RGB_TO_RADAR = "rgb_radar", TOPIC_NAME_OUTPUT_OBSTACLES = "obstacles", 
        TOPIC_NAME_OUTPUT_CLOUD = "cloud";

} // ns


//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleProjectionNodelet::onInit ()
{
    // Call the super onInit ()
    base_type::onInit ();

    NODELET_DEBUG("[%s::onInit] Initializing node", getName().c_str() );

    // advertise publishers
    this->_pub = advertise<asv_perception_common::ObstacleArray>( *pnh_, TOPIC_NAME_OUTPUT_OBSTACLES, 1 );
    this->_pub_cloud = advertise<sensor_msgs::PointCloud2> (*pnh_, TOPIC_NAME_OUTPUT_CLOUD, 1 );

    // get parameters
    float val = 0;  

    if ( pnh_->getParam("min_height", val ) && ( val > 0. ) )
        this->_min_height = val;

    if ( pnh_->getParam("max_height", val ) && ( val > 0. ) )
        this->_max_height = val;

    if ( pnh_->getParam("min_depth", val ) && ( val > 0. ) )
        this->_min_depth = val;

    if ( pnh_->getParam("max_depth", val ) && ( val > 0. ) )
        this->_max_depth = val;

    if ( pnh_->getParam("resolution", val ) && ( val > 0. ) )
        this->_resolution = val;

    if ( pnh_->getParam("min_distance", val ) && ( val > 0. ) )
        this->_min_distance = val;

    if ( pnh_->getParam("max_distance", val ) && ( val > 0. ) )
        this->_max_distance = val;

    pnh_->getParam("roi_grow_limit", this->_roi_grow_limit );
    pnh_->getParam("roi_shrink_limit", this->_roi_shrink_limit );
    pnh_->getParam("use_segmentation", this->_use_segmentation );

    onInitPostProcess ();
}

//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleProjectionNodelet::subscribe ()
{
    static const std::uint32_t SYNC_QUEUE_SIZE = 10;

    std::lock_guard<std::mutex> lg( this->_mtx );

    this->_sub_rgb_radar = pnh_->subscribe<homography_msg_type>( 
        TOPIC_NAME_INPUT_HOMOGRAPHY_RGB_TO_RADAR
        , 1
        , bind( &ObstacleProjectionNodelet::cb_homography_rgb_radar, this, _1 )
    );

    if ( this->_use_segmentation ) { 
    
        this->_sub_segmentation.subscribe( *pnh_, TOPIC_NAME_INPUT_SEGMENTATION, 1 );
        this->_sub_classification.subscribe( *pnh_, TOPIC_NAME_INPUT_CLASSIFICATION, 1 );

        this->_seg_cls_sync.reset(new _seg_cls_synchronizer_type(_seg_cls_sync_policy_type( SYNC_QUEUE_SIZE ),
            _sub_segmentation, _sub_classification));

        this->_seg_cls_sync->registerCallback( bind (&ObstacleProjectionNodelet::sub_callback, this, _1, _2 ) );
    }
    else {  // classification only

        this->_sub_classification_only = pnh_->subscribe<classification_msg_type> (
            TOPIC_NAME_INPUT_CLASSIFICATION, 1, bind( &ObstacleProjectionNodelet::sub_callback, this, _1 ));

    }
}

//////////////////////////////////////////////////////////////////////////////////////////////
void ObstacleProjectionNodelet::unsubscribe ()
{
    std::lock_guard<std::mutex> lg( this->_mtx );

    if ( this->_use_segmentation ) {
        this->_sub_segmentation.unsubscribe();
        this->_sub_classification.unsubscribe();
    } else {
        this->_sub_classification_only.shutdown();
    }

    this->_sub_rgb_radar.shutdown();
}

void ObstacleProjectionNodelet::cb_homography_rgb_radar( const typename homography_msg_type::ConstPtr& h ) {

    if ( !h ) {
        NODELET_WARN( "Invalid homography received, ignoring" );
        return;
    }
    std::lock_guard<std::mutex> lg( this->_mtx );
    this->_h_rgb_radar = h;
}

// segmentation + classification msg
void ObstacleProjectionNodelet::sub_callback ( 
    const typename segmentation_msg_type::ConstPtr& seg_msg, 
    const typename classification_msg_type::ConstPtr& cls_msg)
{

    std::lock_guard<std::mutex> lg( this->_mtx );
    
    // No subscribers/data, no work
    if (!seg_msg
        || !cls_msg 
        || (
            ( this->_pub.getNumSubscribers () < 1 ) 
            && ( this->_pub_cloud.getNumSubscribers() < 1 )
            )
        )
        return;

    // check we have homography, warn
    if ( !this->_h_rgb_radar ) {
        NODELET_WARN( "Homography not yet received, dropping frame" );
        return;
    }

    // we have:
    //  seg img, classifications, homography
    auto img_ptr = cv_bridge::toCvCopy( seg_msg, sensor_msgs::image_encodings::MONO8 );
    
    if ( !img_ptr || img_ptr->image.empty() ) {
        NODELET_WARN( "Invalid segmentation image, dropping frame" );
        return;
    }

    // get cv:Mat reference
    auto& img = img_ptr->image;

    if (
        ( img.cols != cls_msg->image_width )
        || ( img.rows != cls_msg->image_height)
    ) {
        NODELET_WARN( "Segmentation/classification shape mismatch, dropping frame" );
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
        auto msg = asv_perception_common::ObstacleArray();
        
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
            
        this->_pub.publish( msg );

        // debug img
        /*
        if ( this->_pub_debug_img.getNumSubscribers() > 0 ) {
            auto debug_img_msg = sensor_msgs::Image();
            cv_bridge::CvImage debug_img;
            debug_img.encoding = sensor_msgs::image_encodings::MONO8;
            debug_img.image = img;
            debug_img.toImageMsg( debug_img_msg );
            debug_img_msg.header = cls_msg->header;
            this->_pub_debug_img.publish( debug_img_msg );
        }
        */
        
        // unclass pointcloud
        if ( this->_pub_cloud.getNumSubscribers() > 0 ) {

            auto cloud = detail::obstacle_projection::project( 
                img
                , h_rgb_to_radar
                , this->_max_height
                , this->_max_depth
                , this->_resolution
                , this->_max_distance
                );

            sensor_msgs::PointCloud2::Ptr output_blob( new sensor_msgs::PointCloud2() );

            pcl::toROSMsg ( cloud, *output_blob );
            output_blob->header = cls_msg->header;
            output_blob->header.frame_id = child_frame_id;

            // publish
            this->_pub_cloud.publish( output_blob );
        }
  } catch ( const std::exception& ex ) {
    ROS_ERROR("std::exception: %s", ex.what() );
  } catch ( ... ) {
    ROS_ERROR("unknown exception type");
  }
    
}

// classification message only
void ObstacleProjectionNodelet::sub_callback ( 
    const typename classification_msg_type::ConstPtr& cls_msg
    )
{

    std::lock_guard<std::mutex> lg( this->_mtx );
    
    // No subscribers/data, no work
    if ( !cls_msg || this->_pub.getNumSubscribers () < 1 )
        return;
        
    // check we have homography, warn
    if ( !this->_h_rgb_radar ) {
        NODELET_WARN( "Homography not yet received, dropping frame" );
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
        auto msg = asv_perception_common::ObstacleArray();
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
            
        this->_pub.publish( msg );

  } catch ( const std::exception& ex ) {
    ROS_ERROR("std::exception: %s", ex.what() );
  } catch ( ... ) {
    ROS_ERROR("unknown exception type");
  }
    
}

PLUGINLIB_EXPORT_CLASS(obstacle_id::ObstacleProjectionNodelet, nodelet::Nodelet)
