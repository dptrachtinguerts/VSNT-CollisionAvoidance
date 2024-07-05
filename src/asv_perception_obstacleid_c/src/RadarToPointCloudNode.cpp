// Copyright (c) 2020 University of Massachusetts
// All rights reserved.
// This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
// Authors:  Tom Clunie <clunietp@gmail.com>

#include "RadarToPointCloudNode.h"

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace {
    using namespace obstacle_id;

    using point_type = pcl::PointXYZI;
    using pointcloud_type = pcl::PointCloud<point_type>;

    static const std::string 
        TOPIC_NAME_INPUT = "input"
        , TOPIC_NAME_SEGMENT = "segment"
        , TOPIC_NAME_FULL = "full"
        , TOPIC_NAME_FULL_CURRENT = "full_current"
    ;

    /* Fila de segmentos */
    // sums the differences in angles for all segments, assumes clockwise rotation/increasing angles
    float sum_of_angles( const std::deque<std::pair<sensor_msgs::msg::PointCloud2, float>>& segments ) {
        float result = 0.f;
        for ( std::size_t i = 1; i < segments.size(); ++i ) {
        auto current_angle = segments[i].second;
        auto prev_angle = segments[i-1].second;
        if ( prev_angle > current_angle ) // crossover
            prev_angle -= 360.f;
        result += std::abs( current_angle - prev_angle );  // get diff
        }
        return result;
    }

    sensor_msgs::msg::PointCloud2 radarsegment_to_pointcloud( 
        const asv_perception_interfaces::msg::RadarSegment& segment, 
        const float angle_offset, 
        const std::uint8_t min_intensity,
        const std::string& frame_id
    ) {

        pointcloud_type cloud = {};

        for ( const auto& spoke : segment.spokes ) {
        
            const float
                angle = spoke.angle + angle_offset,  // - 270, // correct for orientation forward
                angle_sin = std::sin(angle * M_PI/180.0),
                angle_cos = std::cos(angle * M_PI/180.0),
                max_range = spoke.max_range
                ;
            const int pixels_in_spoke = spoke.data.size();

            // ROS_WARN("spoke angle %s, range %s", std::to_string( angle ).c_str(), std::to_string(max_range).c_str() );

            for ( int i = 0; i < pixels_in_spoke; ++i ) {

                // min intensity check
                if ( spoke.data[i] < min_intensity )
                    continue;

                point_type point = {};
                point.x = max_range * float(i)/float(pixels_in_spoke-1) * angle_sin;
                point.y = max_range * float(i)/float(pixels_in_spoke-1) * angle_cos;
                point.z = 0.0;
                point.intensity=float(spoke.data[i]);
                cloud.push_back(point);
            } // for
        } // for

        auto result = sensor_msgs::msg::PointCloud2();

        pcl::toROSMsg ( cloud, result );
        result.header = segment.header;
        
        if ( !frame_id.empty() )
        result.header.frame_id = frame_id;

        return result;
    } // radarsegment_to_pointcloud
} // ns

RadarToPointCloudNode::RadarToPointCloudNode(const rclcpp::NodeOptions & options) : Node("radar_to_point_cloud", options)
{
    this->declare_parameter("min_intensity", this->min_intensity_);
    this->declare_parameter("frame_id", this->frame_id_ );
    this->declare_parameter("angle_offset", this->angle_offset_ );

    // get parameters
    int val = 0;  
    if ( this->get_parameter("min_intensity", val ) && ( val >= 0 ) )
        this->min_intensity_ = (std::uint8_t)val;

    this->get_parameter("frame_id", this->frame_id_ );
    this->get_parameter("angle_offset", this->angle_offset_ );

    // publishers
    this->pub_segment_ = this->create_publisher<sensor_msgs::msg::PointCloud2>( TOPIC_NAME_SEGMENT, 1 );
    this->pub_full_ = this->create_publisher<sensor_msgs::msg::PointCloud2>( TOPIC_NAME_FULL, 1 );
    this->pub_full_current_ = this->create_publisher<sensor_msgs::msg::PointCloud2>( TOPIC_NAME_FULL_CURRENT, 1 );
}

//////////////////////////////////////////////////////////////////////////////////////////////
void RadarToPointCloudNode::subscribe ()
{
    lock_type_ lg( this->mtx_ );

    this->sub_ = this->create_subscription<asv_perception_interfaces::msg::RadarSegment::SharedPtr>(
        TOPIC_NAME_INPUT
        , 100
        , std::bind (&RadarToPointCloudNode::sub_callback, this, std::placeholders::_1 )
    );
}

//////////////////////////////////////////////////////////////////////////////////////////////
void RadarToPointCloudNode::unsubscribe ()
{
    ;
}

void RadarToPointCloudNode::sub_callback (
    const asv_perception_interfaces::msg::RadarSegment::SharedPtr& segment
)
{
    lock_type_ lg( this->mtx_ );

    try {

        assert( segment.get() );
        auto pc_current = ::radarsegment_to_pointcloud( *segment, this->angle_offset_, this->min_intensity_, this->frame_id_ );

        if ( this->count_subscribers( TOPIC_NAME_SEGMENT ) > 0 )
            this->pub_segment_->publish( pc_current );

        // concat needed?
        if ( ( this->count_subscribers( TOPIC_NAME_FULL ) > 0 ) || ( this->count_subscribers( TOPIC_NAME_FULL_CURRENT ) > 0 ) ) {
            // remove old segments
            /* Se a soma der mais que 360: volta completa */
            while ( ::sum_of_angles( this->segments_ ) > 360.f )
                this->segments_.pop_front(); /* Retira do início da lista */

            assert( segment->spokes.size() > 0 );

            const auto current_angle = std::abs(segment->spokes[0].angle);
            /* Por que move? */
            this->segments_.emplace_back( std::move(pc_current), current_angle ); // pc_current moved

            // are we at crossover point?  assumes input radar segments are clockwise
            const bool at_crossover = ( ( this->segments_.size() > 1 ) && ( ( current_angle - this->segments_[this->segments_.size()-2].second ) < 0.f ) );

            // pointcloud concat
            auto result = sensor_msgs::msg::PointCloud2();
            for ( auto& pc_pair : this->segments_ ) {
                auto& pc = pc_pair.first;
                if ( ( pc.width > 0 ) && !pcl::concatenatePointCloud( result, pc, result ) ) {
                    RCLCPP_ERROR(this->get_logger(), "Error concatenating point clouds");
                }
            }
            
            // use latest header
            result.header = this->segments_.back().first.header;

            if ( this->count_subscribers(TOPIC_NAME_FULL_CURRENT) > 0 ) {
                this->pub_full_current_->publish( result );
            }
            
            // print 'full' if at crossover point
            if ( at_crossover && ( this->count_subscribers(TOPIC_NAME_FULL) > 0 ) ) {
                this->pub_full_->publish( result );
            }
        }

    } catch ( const std::exception& ex ) {  // pcl exceptions inherit from std::runtime_error
        RCLCPP_ERROR(this->get_logger(), "std::exception: %s", ex.what() );
    } catch ( ... ) {
        RCLCPP_ERROR(this->get_logger(), "unknown exception type");
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE( obstacle_id::RadarToPointCloudNode)