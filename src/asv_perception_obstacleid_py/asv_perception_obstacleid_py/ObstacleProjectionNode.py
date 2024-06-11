




import rclpy
from rclpy.node import Node
from asv_perception_interfaces import ClassificationArray, Homography

class ObstacleProjectionNode(Node):
    '''
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
    '''

    def __init__(self):
        super().__init__("ObstacleProjectionNode")

        # Message names
        self.TOPIC_NAME_INPUT_SEGMENTATION = "segmentation"
        self.TOPIC_NAME_INPUT_CLASSIFICATION = "classification"
        self.TOPIC_NAME_INPUT_HOMOGRAPHY_RGB_TO_RADAR = "rgb_radar"
        self.TOPIC_NAME_OUTPUT_OBSTACLES = "obstacles"
        self.TOPIC_NAME_OUTPUT_CLOUD = "cloud"

        min_height_default = 1.
        max_height_default = 1.
        min_depth_default = 1.
        max_depth_default = 1.
        resolution_default = 0.2
        min_distance_default = 3.
        max_distance_default = 100.
        roi_grow_limit_default = 0.
        roi_shrink_limit_default = 0.
        use_segmentation_default = False         

        # Default parameters 
        self.declare_parameter("min_height", min_height_default)
        self.declare_parameter("max_height", max_height_default)
        self.declare_parameter("min_depth", min_depth_default)
        self.declare_parameter("max_depth", max_depth_default)
        self.declare_parameter("resolution", resolution_default)
        self.declare_parameter("min_distance", min_distance_default)
        self.declare_parameter("max_distance", max_distance_default)
        self.declare_parameter("roi_grow_limit", roi_grow_limit_default)
        self.declare_parameter("roi_shrink_limit", roi_shrink_limit_default)
        self.declare_parameter("use_segmentation", use_segmentation_default)

        # Get parameters from launch file TODO: voltar para if
        if self.get_parameter("min_height").value > 0:
            self._min_height = 
        if self.get_parameter("max_height").value > 0:
            self._max_height = 
        if self.get_parameter("min_depth").value > 0:
            self._min_depth = 
        if self.get_parameter("max_depth").value > 0:
            self._max_depth = 
        if self.get_parameter("resolution").value > 0:
            self._resolution = 
        if self.get_parameter("min_distance").value > 0:
            self._min_distance = 
        if self.get_parameter("max_distance").value > 0:
            self._max_distance = 
        if self.get_parameter("roi_grow_limit").value > 0:
            self._roi_grow_limit = 
        if self.get_parameter("roi_shrink_limit").value > 0:
            self._roi_shrink_limit = 
        if self.get_parameter("use_segmentation").value > 0:
            self._use_segmentation = 

        # TODO: add segmentation
        self._use_segmentation = False

    def subscribe(self):
        self._homography_sub = self.create_subscription(
            Homography,
            self.TOPIC_NAME_INPUT_HOMOGRAPHY_RGB_TO_RADAR,
            self.cb_homography_rgb_radar, 
            10
        )

        if self._use_segmentation:
            print("Not implemented yet")
        else:
            # Classification only


    def unsubscribe(self):
        pass

    def cb_classification(self, cls_arr_msg):
        '''
        cls_arr: ClassificationArray message
        '''
        pass

    def cb_homography_rgb_radar(self, hgp_msg):
        '''
        hgp_msg: Homography message
        '''
        pass