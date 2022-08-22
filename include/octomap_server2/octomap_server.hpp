
#pragma once
#ifndef _OCTOMAP_SERVER_HPP_
#define _OCTOMAP_SERVER_HPP_

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rmw/qos_profiles.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf2/buffer_core.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/srv/bounding_box_query.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <octomap_server2/transforms.hpp>
#include <octomap_server2/conversions.h>


using PCLPoint = pcl::PointXYZ;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;
using OcTreeT = octomap::OcTree;

using OctomapSrv =  octomap_msgs::srv::GetOctomap;
using BBXSrv =  octomap_msgs::srv::BoundingBoxQuery;


namespace ph = std::placeholders;

namespace octomap_server {
    class OctomapServer: public rclcpp::Node {
   
    public:	    
        std::shared_ptr<OcTreeT> m_octree;
    protected:

        std::shared_ptr<message_filters::Subscriber<
                            sensor_msgs::msg::PointCloud2>> m_pointCloudSub;
        std::shared_ptr<tf2_ros::MessageFilter<
                            sensor_msgs::msg::PointCloud2>> m_tfPointCloudSub;
        
        static std_msgs::msg::ColorRGBA heightMapColor(double h);

        rclcpp::Publisher<visualization_msgs::msg::MarkerArray
                          >::SharedPtr m_fmarkerPub;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray
                          >::SharedPtr m_markerPub;
        rclcpp::Publisher<octomap_msgs::msg::Octomap
                          >::SharedPtr m_binaryMapPub;
        rclcpp::Publisher<octomap_msgs::msg::Octomap
                          >::SharedPtr m_fullMapPub;
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid
                          >::SharedPtr m_mapPub;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2
		          >::SharedPtr m_occupiedPCLPub;

	rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Service<OctomapSrv>::SharedPtr m_octomapBinaryService;
        rclcpp::Service<OctomapSrv>::SharedPtr m_octomapFullService;
        rclcpp::Service<std_srvs::srv::Empty>::SharedPtr m_resetService;

        std::shared_ptr<tf2_ros::Buffer> buffer_;
        std::shared_ptr<tf2_ros::TransformListener> m_tfListener;

        
        double m_minRange;
        double m_maxRange;
        std::string m_worldFrameId; // the map frame
        bool m_useHeightMap;
        std_msgs::msg::ColorRGBA m_color;
        std_msgs::msg::ColorRGBA m_colorFree;
        double m_colorFactor;
        
        bool m_publishFreeSpace;
        bool m_publishMarkerArray;
        bool m_publishBinaryMap;
        bool m_publishFullMap;
	bool m_publishOccupiedPCL;
	int m_publishEvery_ms;
	double m_publishResolution;

        double m_res;
        unsigned m_treeDepth;
        unsigned m_maxTreeDepth;
        double m_pointcloudMinX;
        double m_pointcloudMaxX;
        double m_pointcloudMinY;
        double m_pointcloudMaxY;
        double m_pointcloudMinZ;
        double m_pointcloudMaxZ;
        double m_occupancyMinZ;
        double m_occupancyMaxZ;
        
	bool m_compressMap;

        // downprojected 2D map:
        bool m_mapOriginChanged;
        octomap::OcTreeKey m_paddedMinKey;
        unsigned m_multires2DScale;
        bool m_projectCompleteMap;
       
	int print_logger =0;

        void publishMarkerArrayOctomap(const rclcpp::Time &) const;
        void publishBinaryOctoMap(const rclcpp::Time &) const;
        void publishFullOctoMap(const rclcpp::Time &) const;
        void publishAllNow();
        virtual void publishAll(const rclcpp::Time &);
        
        virtual void insertScanPointCloud(
            const geometry_msgs::msg::Vector3 &sensorOriginTf,
	    const PCLPointCloud& pc);

        virtual void onInit();        
        virtual void subscribe();
        
        
    public:        
        explicit OctomapServer(
            const std::string = "octomap_server");
        //explicit OctomapServer(
        //    const rclcpp::NodeOptions &,
        //    const std::string = "octomap_server");
        virtual ~OctomapServer();        
        virtual bool octomapBinarySrv(
            const std::shared_ptr<OctomapSrv::Request> ,
            std::shared_ptr<OctomapSrv::Response>);
        virtual bool octomapFullSrv(
            const std::shared_ptr<OctomapSrv::Request> ,
            std::shared_ptr<OctomapSrv::Response>);

        bool resetSrv(
            const std::shared_ptr<std_srvs::srv::Empty::Request>,
            std::shared_ptr<std_srvs::srv::Empty::Response>);

        virtual void insertCloudCallback(
            const sensor_msgs::msg::PointCloud2::ConstSharedPtr &);
        virtual bool openFile(const std::string& filename);

        virtual std::shared_ptr<OcTreeT> get_octree() { return m_octree; };
    };    
} // octomap_server

#endif  // _OCTOMAP_SERVER_HPP_
