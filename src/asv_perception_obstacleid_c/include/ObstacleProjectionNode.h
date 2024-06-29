#ifndef OBSTACLEPROJECTIONNODE_H
#define OBSTACLEPROJECTIONNODE_H

#include <rclcpp/rclcpp.hpp>
#include "asv_perception_interfaces/msg/classification_array.hpp"
#include "asv_perception_interfaces/msg/homography.hpp"
#include "asv_perception_interfaces/msg/obstacle_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <mutex>

#include "ClassifiedObstacle2d.h"

namespace obstacle_id {
    /*
    Nodelet for obstacle backprojection from 2D to 3D.  
    In 2D, combines segmented obstacle map with classified obstacle bounding boxes
    Expands classified obstacle bounding boxes as needed, estimates 3d properties, then creates Obstacle messages for the classified obstacles
    Projects remaining unclassified obstacle pixels to pointcloud
    Subscriptions:
        ~segmentation:      [sensor_msgs/Image] 2d obstacle map, all pixels with value > 0 are considered obstacle pixels
        ~classification:    [asv_perception_common/ClassificationArray]  Classifications
        ~rgb_radar:         [asv_perception_common/Homography] rgb to radar homography matrix
    
    Publications:
        ~obstacles:         [asv_perception_common/ObstacleArray] Classified obstacles
        ~cloud:             [sensor_msgs/PointCloud2]  Unclassified obstacle pointcloud, if segmentation is enabled

    Parameters:
        ~use_segmentation   [bool, default=true]  Use segmentation image
        ~min_height         [float, default=1.0]  Minimum projected obstacle height
        ~max_height         [float, default=1.0]  Maximum projected obstacle height
        ~min_depth          [float, default=1.0]  Minimum projected obstacle depth
        ~max_depth          [float, default=1.0]  Maximum projected obstacle depth
        ~resolution         [float, default=0.25]  Obstacle pointcloud resolution (space between points)
        ~min_distance       [float, default=3.0]  Minimum projected obstacle distance
        ~max_distance       [float, default=100.0]  Maximum projected obstacle distance
        ~roi_shrink_limit   [float, default=0]    Percentage limit of how much a classified obstacle ROI can shrink
        ~roi_grow_limit     [float, default=0]    Percentage limit of how much a classified obstacle ROI can grow
    */
    class ObstacleProjectionNode : public rclcpp::Node {
        public:
            using classification_msg_t = asv_perception_interfaces::msg::ClassificationArray;
            using homography_msg_t = asv_perception_interfaces::msg::Homography;
            using obstacle_arr_msg_t = asv_perception_interfaces::msg::ObstacleArray;

            ObstacleProjectionNode(const rclcpp::NodeOptions & options);
        protected:
            void subscribe(void);
            void unsubscribe(void);

            void cb_sub_classification(const classification_msg_t::SharedPtr&);

            void cb_homography_rgb_radar(const homography_msg_t::SharedPtr&);

        private:
            std::mutex _mtx;

            rclcpp::Publisher<obstacle_arr_msg_t>::SharedPtr _pub_obstacle_arr;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub_cloud;

            rclcpp::Subscription<homography_msg_t::SharedPtr>::SharedPtr _sub_rgb_radar;
            rclcpp::Subscription<classification_msg_t::SharedPtr>::SharedPtr _sub_classification_only;

            homography_msg_t::SharedPtr _h_rgb_radar;
 
            std::float_t _min_height = 1.f;
            std::float_t _max_height = 1.f;
            std::float_t _min_depth = 1.f;
            std::float_t _max_depth = 1.f;
            std::float_t _resolution = 0.25f;
            std::float_t _min_distance = 3.f;
            std::float_t _max_distance = 100.f;
            std::float_t _roi_grow_limit = 0.f;
            std::float_t _roi_shrink_limit = 0.f;

            bool _use_segmentation = false;
    }; 
}

#endif
