// Copyright (c) 2020 University of Massachusetts
// All rights reserved.
// This source code is licensed under the BSD-style license found in the LICENSE file in the root directory of this source tree.
// Authors:  Tom Clunie <clunietp@gmail.com>

#ifndef OBSTACLE_EXTRACTION_NODELET_H
#define OBSTACLE_EXTRACTION_NODELET_H

#include <rclcpp/rclcpp.hpp>
#include "asv_perception_interfaces/msg/obstacle_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "defs.h"

namespace obstacle_id
{
    /*
    Obstacle extraction nodelet performs euclidean clustering and constructs Obstacle messages

    Parameters:
        ~cluster_tolerance: [float]               clustering tolerance
        ~cluster_size_min:  [uint, default=1]     minimum number of points in a cluster
        ~cluster_size_max:  [uint, default=max]   maximum number of points in a cluster
        ~cluster_area_min:  [float, default=0]    minimum convex hull area
        ~cluster_area_max:  [float, default=max]  maximum convex hull area
    
    Topics:
        ~input:      [sensor_msgs/PointCloud2]               input pointcloud
        ~obstacles:  [asv_perception_common/ObstacleArray]   output obstacles
    */
    class ObstacleExtractionNode : public rclcpp::Node {
        public:
            // default constructor
            ObstacleExtractionNode(const rclcpp::NodeOptions & options);
                                        
        protected:
            /* Cria limites para o número de pontos no 
            cluster = 1 até limite uint32_t (4,294,967,295) */
            std::uint32_t 
            cluster_sz_min_ = 1
            , cluster_sz_max_ = std::numeric_limits<std::uint32_t>::max()
            ;

            float
                cluster_tolerance_ = 0.f
                , cluster_area_min_ = 0.f
                , cluster_area_max_ = std::numeric_limits<float>::max()
            ;

            /** \brief LazyNodelet connection routine. */
            void subscribe(void);
            void unsubscribe(void);

            // the callback function to handle input from subscription
            void sub_callback ( const sensor_msgs::msg::PointCloud2::SharedPtr& );
            
        private:
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
            rclcpp::Publisher<asv_perception_interfaces::msg::ObstacleArray>::SharedPtr pub_;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif  //#ifndef 