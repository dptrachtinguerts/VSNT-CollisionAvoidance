#ifndef CLASSIFIED_OBSTACLE_PROJECTION_H
#define CLASSIFIED_OBSTACLE_PROJECTION_H

#include <asv_perception_interfaces/msg/obstacle.hpp>
#include <asv_perception_interfaces/msg/classification_array.hpp>
#include <asv_perception_interfaces/msg/homography.hpp>

#include "defs.h"

namespace obstacle_id {
namespace detail {
namespace classified_obstacle_projection {

    using namespace asv_perception_interfaces::msg;

    namespace impl {
        ;
    } // impl ns

    /*
    Combines an unclassified obstacle map (optional) with a vector of classification bounding boxes
        Expand classification bounding boxes as needed and create parent/child relationships
        Classified obstacle bounding boxes are then removed from the provided obstacle map
        Returns vector of projected Obstacles
    */
    inline std::vector<Obstacle> project( 
        image_type& obstacle_map,
        const ClassificationArray& classifications,
        const Homography& h,
        const float min_height, const float max_height,
        const float min_depth, const float max_depth,
        const float min_distance, const float max_distance,
        const float roi_grow_limit = 0.f,
        const float roi_shrink_limit = 0.f) 
    {
        if (!obstacle_map.empty() && (classifications.image_height != obstacle_map.rows || classifications.image_width != obstacle_map.cols)) {
            throw std::runtime_error("obstacle map not empty and image size does not match classification image size");
        }

        // auto 
    }

} // classified_obstacle_projection ns
} // detail ns
} // obstacle_id ns


#endif