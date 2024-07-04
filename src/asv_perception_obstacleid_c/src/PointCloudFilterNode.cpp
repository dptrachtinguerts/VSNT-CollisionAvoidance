// Copyright (c) 2020 University of Massachusetts
// All rights reserved.
// This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
// Authors:  Tom Clunie <clunietp@gmail.com>

#include "PointCloudFilterNode.h"
#include <math.h> // pow

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <rclcpp_components/register_node_macro.hpp>

#include "utils.h"
#include "detail/PointCluster.h"

namespace {
	using namespace obstacle_id;

	static const std::string 
			TOPIC_NAME_INPUT = "input"
			, TOPIC_NAME_OUTPUT = "output"
	;
} // ns

PointCloudFilterNode::PointCloudFilterNode(const rclcpp::NodeOptions & options) : Node("point_cloud_filter", options)
{
	// parameters
	this->declare_parameter("max_distance", this->max_distance_ );
	this->declare_parameter("min_distance", this->min_distance_ );
	this->declare_parameter("min_distance_x", this->min_distance_x_ );
	this->declare_parameter("min_distance_y", this->min_distance_y_ );
	this->declare_parameter("min_distance_z", this->min_distance_z_ );
	this->declare_parameter("outlier_min_neighbors", this->outlier_min_neighbors_ );
	this->declare_parameter("outlier_radius", this->outlier_radius_ );
	
	this->get_parameter("max_distance", this->max_distance_ );
	this->get_parameter("min_distance", this->min_distance_ );
	this->get_parameter("min_distance_x", this->min_distance_x_ );
	this->get_parameter("min_distance_y", this->min_distance_y_ );
	this->get_parameter("min_distance_z", this->min_distance_z_ );
	this->get_parameter("outlier_min_neighbors", this->outlier_min_neighbors_ );
	this->get_parameter("outlier_radius", this->outlier_radius_ );

	int val = 0;  
	if ( this->get_parameter("cluster_size_max", val ) && ( val >= 0 ) ) {
		this->cluster_sz_max_ = (std::uint32_t)val;
	}

	if ( this->get_parameter("cluster_size_min", val ) && ( val >= 0 ) ) {
		this->cluster_sz_min_ = (std::uint32_t)val;
	}

	this->get_parameter("cluster_tolerance", this->cluster_tolerance_ );
	this->get_parameter("cluster_area_max", this->cluster_area_max_ );
	this->get_parameter("cluster_area_min", this->cluster_area_min_ );
	this->get_parameter("cluster_inliers", this->cluster_inliers_ );

	// publisher
	this->pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(TOPIC_NAME_OUTPUT, 1);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudFilterNode::subscribe ()
{
	this->sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2::SharedPtr> (
		TOPIC_NAME_INPUT
		, 1
		, std::bind(&PointCloudFilterNode::sub_callback, this, std::placeholders::_1 )
	);
}

//////////////////////////////////////////////////////////////////////////////////////////////
void PointCloudFilterNode::unsubscribe ()
{
	;
}

void PointCloudFilterNode::sub_callback (
		const sensor_msgs::msg::PointCloud2::SharedPtr& cloud)
{
	using pointcloud_type = pcl::PointCloud<point_type>;
	using point_type = pcl::PointXYZ;
	// No subscribers, no work
	if ( this->count_subscribers(TOPIC_NAME_OUTPUT) < 1 ) {
		return;
	}

	// empty cloud, no work
	if ( cloud->data.empty() ) {
		return;
	}

	try {
		pointcloud_type::Ptr pc_ptr(new pointcloud_type());

		pcl::fromROSMsg( *cloud, *pc_ptr );
		if ( pc_ptr->empty() )
			return;

		// clustering filter
		if ( this->cluster_tolerance_ > 0.f ) {

			pointcloud_type::Ptr filtered( new pointcloud_type{} );
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

			pcl::ExtractIndices<point_type> extract = {};
			/* Para cada cluster... */
			for ( const auto& cluster : detail::PointCluster::extract( 
				pc_ptr
				, this->cluster_tolerance_
				, this->cluster_sz_min_
				, this->cluster_sz_max_
				, this->cluster_area_min_
				, this->cluster_area_max_)) 
			{	
				/* Se o ponto da PC estiver dentro do cluster, adicionar seu índice à 
						lista de inliers */
				for ( const auto idx : cluster.indices.indices )
					inliers->indices.push_back(idx);
			}

			extract.setInputCloud( pc_ptr );
			extract.setIndices(inliers);

			/* Ao setar cluster_inliers_ como true, os inliers são retirados da 
					filtragem */
			extract.setNegative( !this->cluster_inliers_ );  // setNegative(true) = remove the inliers
			extract.filter(*pc_ptr);
		}
		/* Se o ponto está fora do raio de filtragem, colocá-lo nos inliers */

		// max distance filter (radius from origin)
		if ( !pc_ptr->empty() && !std::isnan( this->max_distance_ ) ) {

			const auto d_2 = std::pow( this->max_distance_, 2. );
			pointcloud_type::Ptr filtered( new pointcloud_type{} );
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

			pcl::ExtractIndices<point_type> extract = {};
			for ( std::size_t i = 0; i < pc_ptr->points.size(); ++i ) {
				const auto& pt = pc_ptr->points[i];
				if ( ( std::pow(pt.x,2.) + std::pow(pt.y,2.) + std::pow(pt.z,2.) ) > d_2 )
					inliers->indices.push_back(i);
			}
			extract.setInputCloud( pc_ptr );
			extract.setIndices(inliers);
			extract.setNegative(true);
			extract.filter(*pc_ptr);
		}

		/* Se o ponto está fora do raio de filtragem, colocá-lo nos inliers */

		// min distance filter (radius from origin)
		if ( !pc_ptr->empty() && ( this->min_distance_ > 0.f ) ) {

			const auto d_2 = std::pow( this->min_distance_, 2. );

			// https://stackoverflow.com/a/48595186/882436
			pointcloud_type::Ptr filtered( new pointcloud_type{} );
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
			pcl::ExtractIndices<point_type> extract = {};
			for ( std::size_t i = 0; i < pc_ptr->points.size(); ++i ) {
				const auto& pt = pc_ptr->points[i];
				if ( ( std::pow(pt.x,2.) + std::pow(pt.y,2.) + std::pow(pt.z,2.) ) < d_2 )
					inliers->indices.push_back(i);
			}
			extract.setInputCloud( pc_ptr );
			extract.setIndices(inliers);
			extract.setNegative(true);
			extract.filter(*pc_ptr);
		}

		/* Filtro em forma de paralelepípedo, cujo centro é o radar e 
				2*min_distance_x_, 2*min_distance_y_ e 2*min_distance_z_ são as dimensões */

		// dimensional min distance filter
		if ( !pc_ptr->empty() 
				&&
				( ( this->min_distance_x_ > 0.f ) || ( this->min_distance_y_ > 0.f ) || ( this->min_distance_z_ > 0.f ) ) 
		) {
			using PointType = point_type;
			pcl::ConditionOr<PointType>::Ptr range_cond (new pcl::ConditionOr<PointType> ());

			if ( this->min_distance_x_ > 0.f ) {
				range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
																				pcl::FieldComparison<PointType> ("x", pcl::ComparisonOps::GT, this->min_distance_x_ )));
				range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
																				pcl::FieldComparison<PointType> ("x", pcl::ComparisonOps::LT, -this->min_distance_x_ )));
			}

			if ( this->min_distance_y_ > 0.f ) {
				range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
																				pcl::FieldComparison<PointType> ("y", pcl::ComparisonOps::GT, this->min_distance_y_ )));
				range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
																				pcl::FieldComparison<PointType> ("y", pcl::ComparisonOps::LT, -this->min_distance_y_ )));
			}

			if ( this->min_distance_z_ > 0.f ) {
				range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
																				pcl::FieldComparison<PointType> ("z", pcl::ComparisonOps::GT, this->min_distance_z_ )));
				range_cond->addComparison (pcl::FieldComparison<PointType>::ConstPtr (new
																				pcl::FieldComparison<PointType> ("z", pcl::ComparisonOps::LT, -this->min_distance_z_ )));
			}

			pointcloud_type::Ptr cloud_post_filter (new pointcloud_type);
			pcl::ConditionalRemoval<PointType> condrem;
			condrem.setCondition(range_cond);
			condrem.setInputCloud(pc_ptr);
			condrem.filter(*cloud_post_filter);
			pc_ptr = cloud_post_filter;
		}

		/* Remove pontos isolados */

		// radius outlier removal
		if ( !pc_ptr->empty() && ( this->outlier_min_neighbors_ > 0 ) && ( this->outlier_radius_ > 0. ) ) {
			pcl::RadiusOutlierRemoval<point_type> outrem = {};
			outrem.setInputCloud(pc_ptr);
			outrem.setRadiusSearch(this->outlier_radius_);
			outrem.setMinNeighborsInRadius(this->outlier_min_neighbors_);
			outrem.filter(*pc_ptr);
		}

		// publish
		this->pub_->publish( pc_ptr );

	} catch ( const std::exception& ex ) {  // pcl exceptions inherit from std::runtime_error
		RCLCPP_ERROR(this->get_logger(), "std::exception: %s", ex.what() );
	} catch ( ... ) {
		RCLCPP_ERROR(this->get_logger(), "unknown exception type");
	}
}

RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_id::PointCloudFilterNode)