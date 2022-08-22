
#include <octomap_server2/octomap_server.hpp>

namespace octomap_server {
    OctomapServer::OctomapServer(
        //const rclcpp::NodeOptions &options,
        const std::string node_name):
        //Node(node_name, options),
        Node(node_name),
	m_octree(NULL),
        m_maxRange(20),
	m_minRange(0.5),
        m_worldFrameId("/map"),
        m_useHeightMap(true),
        m_colorFactor(0.8),
        m_publishFreeSpace(false),
        m_res(0.05),
        m_treeDepth(0),
        m_maxTreeDepth(0),
	m_publishResolution(0.05),
        m_pointcloudMinX(-std::numeric_limits<double>::max()),
        m_pointcloudMaxX(std::numeric_limits<double>::max()),
        m_pointcloudMinY(-std::numeric_limits<double>::max()),
        m_pointcloudMaxY(std::numeric_limits<double>::max()),
        m_pointcloudMinZ(-std::numeric_limits<double>::max()),
        m_pointcloudMaxZ(std::numeric_limits<double>::max()),
        m_occupancyMinZ(-std::numeric_limits<double>::max()),
        m_occupancyMaxZ(std::numeric_limits<double>::max()),
        m_compressMap(true),
        m_publishEvery_ms(100)

	{

        rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
        this->buffer_ = std::make_shared<tf2_ros::Buffer>(clock);
        this->buffer_->setUsingDedicatedThread(true);
        this->m_tfListener = std::make_shared<tf2_ros::TransformListener>(
            *buffer_, this, false);
        
        m_worldFrameId = this->declare_parameter("frame_id", m_worldFrameId);
        m_useHeightMap = this->declare_parameter("height_map", m_useHeightMap);
        m_colorFactor = this->declare_parameter("color_factor", m_colorFactor);

	m_publishEvery_ms = this->declare_parameter("publish_every_ms", m_publishEvery_ms);
	m_publishResolution = this->declare_parameter("publish_resolution_m", m_publishResolution);

        m_pointcloudMinX = this->declare_parameter(
            "pointcloud_min_x", m_pointcloudMinX);
        m_pointcloudMaxX = this->declare_parameter(
            "pointcloud_max_x", m_pointcloudMaxX);
        m_pointcloudMinY = this->declare_parameter(
            "pointcloud_min_y", m_pointcloudMinY);
        m_pointcloudMaxY = this->declare_parameter(
            "pointcloud_max_y", m_pointcloudMaxY);
        m_pointcloudMinZ = this->declare_parameter(
            "pointcloud_min_z", m_pointcloudMinZ);
        m_pointcloudMaxZ = this->declare_parameter(
            "pointcloud_max_z", m_pointcloudMaxZ);
        m_occupancyMinZ = this->declare_parameter(
            "occupancy_min_z", m_occupancyMinZ);
        m_occupancyMaxZ = this->declare_parameter(
            "occupancy_max_z", m_occupancyMaxZ);
        
	m_maxRange = this->declare_parameter(
            "sensor_model/max_range", m_maxRange);

        m_minRange = this->declare_parameter(
            "sensor_model/min_range", m_minRange);
        
	m_res = this->declare_parameter("resolution", m_res);
        
	double probHit = this->declare_parameter("sensor_model/hit", 0.7);
        double probMiss = this->declare_parameter("sensor_model/miss", 0.4);
        double thresMin = this->declare_parameter("sensor_model/min", 0.12);
        double thresMax = this->declare_parameter("sensor_model/max", 0.97);
        
	m_compressMap = this->declare_parameter("compress_map", m_compressMap);
        
        // initialize octomap object & params
        m_octree = std::make_shared<OcTreeT>(m_res);
        m_octree->setProbHit(probHit);
        m_octree->setProbMiss(probMiss);
        m_octree->setClampingThresMin(thresMin);
        m_octree->setClampingThresMax(thresMax);
        m_treeDepth = m_octree->getTreeDepth();
        m_maxTreeDepth = m_treeDepth;

        double r = this->declare_parameter("color/r", 0.0);
        double g = this->declare_parameter("color/g", 0.0);
        double b = this->declare_parameter("color/b", 1.0);
        double a = this->declare_parameter("color/a", 1.0);
        m_color.r = r;
        m_color.g = g;
        m_color.b = b;
        m_color.a = a;

        r = this->declare_parameter("color_free/r", 0.0);
        g = this->declare_parameter("color_free/g", 1.0);
        b = this->declare_parameter("color_free/b", 0.0);
        a = this->declare_parameter("color_free/a", 1.0);
        m_colorFree.r = r;
        m_colorFree.g = g;
        m_colorFree.b = b;
        m_colorFree.a = a;
        
        m_publishFreeSpace = this->declare_parameter(
            "publish_free_space", m_publishFreeSpace);
        m_publishMarkerArray = this->declare_parameter(
            "publish_marker_array", m_publishMarkerArray);
        m_publishBinaryMap = this->declare_parameter(
            "publish_binary_map", m_publishBinaryMap);
        m_publishFullMap = this->declare_parameter(
            "publish_full_map", m_publishFullMap);
	m_publishOccupiedPCL = this->declare_parameter(
	    "publish_occupied_pcl", m_publishOccupiedPCL);

        std::string msg = std::string("Publishing non-latched (topics are only)") +
                    "prepared as needed, will only be re-published on map change";
        RCLCPP_INFO(this->get_logger(), msg.c_str());

        RCLCPP_INFO(this->get_logger(), "Frame Id %s", m_worldFrameId.c_str());
        RCLCPP_INFO(this->get_logger(), "Resolution %.2f", m_res);
        
        this->onInit();
    }

    OctomapServer::~OctomapServer() {

    }

    void OctomapServer::onInit() {
        this->subscribe();

        //rclcpp::QoS qos(rclcpp::KeepLast(3));
        
	rclcpp::QoS qos(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
	
	this->m_markerPub = this->create_publisher<
            visualization_msgs::msg::MarkerArray>(
                "occupied_cells_vis_array", qos);
        
        this->m_binaryMapPub = this->create_publisher<
            octomap_msgs::msg::Octomap>("octomap_binary", qos);
        
        this->m_fullMapPub = this->create_publisher<
            octomap_msgs::msg::Octomap>("octomap_full", qos);
        
        this->m_mapPub = this->create_publisher<
            nav_msgs::msg::OccupancyGrid>("projected_map", qos);
        
        this->m_fmarkerPub = this->create_publisher<
            visualization_msgs::msg::MarkerArray>(
                "free_cells_vis_array", qos);

	this->m_occupiedPCLPub = this->create_publisher<
            sensor_msgs::msg::PointCloud2>(
		"occupied_pcl", qos);

	//this->timer_ = this->create_wall_timer(
	//		std::chrono::milliseconds(m_publishEvery_ms),
	//		std::bind(&OctomapServer::publishAllNow, this)
	//		);
    }

    void OctomapServer::subscribe() {
        this->m_pointCloudSub = std::make_shared<
            message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
                this, "cloud_in", rmw_qos_profile_sensor_data);

        auto create_timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface());
        
        this->buffer_->setCreateTimerInterface(create_timer_interface);
        
        this->m_tfPointCloudSub = std::make_shared<tf2_ros::MessageFilter<
            sensor_msgs::msg::PointCloud2>>(
                *buffer_, m_worldFrameId, 5,
                this->get_node_logging_interface(),
                this->get_node_clock_interface(),
                std::chrono::seconds(1));
        
        this->m_tfPointCloudSub->connectInput(*m_pointCloudSub);
        
        this->m_tfPointCloudSub->registerCallback(
            std::bind(&OctomapServer::insertCloudCallback, this, ph::_1));

        this->m_octomapBinaryService = this->create_service<OctomapSrv>(
            "octomap_binary",
            std::bind(&OctomapServer::octomapBinarySrv, this, ph::_1, ph::_2));
        
	this->m_octomapFullService = this->create_service<OctomapSrv>(
            "octomap_full",
            std::bind(&OctomapServer::octomapFullSrv, this, ph::_1, ph::_2));
        
	this->m_resetService = this->create_service<std_srvs::srv::Empty>(
            "reset", std::bind(&OctomapServer::resetSrv, this, ph::_1, ph::_2));

        RCLCPP_INFO(this->get_logger(), "Setup completed!");
    }

    bool OctomapServer::openFile(const std::string &filename){
        if (filename.length() <= 3)
            return false;

        std::string suffix = filename.substr(filename.length()-3, 3);
        if (suffix== ".bt") {
            if (!m_octree->readBinary(filename)) {
                return false;
            }
        } else if (suffix == ".ot") {
            auto tree = octomap::AbstractOcTree::read(filename);
            if (!tree){
                return false;
            }

            OcTreeT *octree = dynamic_cast<OcTreeT*>(tree);
            m_octree = std::shared_ptr<OcTreeT>(octree);
            
            if (!m_octree) {
                std::string msg = "Could not read OcTree in file";
                RCLCPP_ERROR(this->get_logger(), msg.c_str());
                return false;
            }
        } else {
            return false;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Octomap file %s loaded (%zu nodes).",
                    filename.c_str(), m_octree->size());

        m_treeDepth = m_octree->getTreeDepth();
        m_maxTreeDepth = m_treeDepth;
        m_res = m_octree->getResolution();

        rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>();
        publishAll(clock->now());
        return true;
    }

    void OctomapServer::insertCloudCallback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud){
        auto start = std::chrono::steady_clock::now();
      
        
        // ground filtering in base frame
        PCLPointCloud pc; // input cloud for filtering and ground-detection
        pcl::fromROSMsg(*cloud, pc);
       
        // get transform to world coordinates	
        Eigen::Matrix4f sensorToWorld;
        geometry_msgs::msg::TransformStamped sensorToWorldTf;
        try {
            if (!this->buffer_->canTransform(
                    m_worldFrameId, cloud->header.frame_id,
                    cloud->header.stamp)) {
                throw "Failed";
            }
            
            sensorToWorldTf = this->buffer_->lookupTransform(
                m_worldFrameId, cloud->header.frame_id,
                cloud->header.stamp);
            sensorToWorld = pcl_ros::transformAsMatrix(sensorToWorldTf);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "%s",ex.what());
            return;
        }

            
        // directly transform to map frame:
        pcl::transformPointCloud(pc, pc, sensorToWorld);
        
        // set up filter for height range, also removes NANs:
        pcl::PassThrough<PCLPoint> pass_x;
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
        pcl::PassThrough<PCLPoint> pass_y;
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
        pcl::PassThrough<PCLPoint> pass_z;
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);
        
	// just filter height range:
        pass_x.setInputCloud(pc.makeShared());
        pass_x.filter(pc);
        pass_y.setInputCloud(pc.makeShared());
        pass_y.filter(pc);
        pass_z.setInputCloud(pc.makeShared());
        pass_z.filter(pc);

	insertScanPointCloud(sensorToWorldTf.transform.translation, pc);

        auto end = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        RCLCPP_INFO(this->get_logger(), "Time lapse %f, octomap_size %zu", elapsed_seconds.count(), m_octree->size());
        
	//publishAll(cloud->header.stamp);
    }

    void OctomapServer::insertScanPointCloud(
        const geometry_msgs::msg::Vector3 &sensorOriginTf,
	const PCLPointCloud& pc){
      

	// convert pc to octomap pc;
	octomap::Pointcloud scan;
	scan.reserve(pc.size());
	for (auto it = pc.begin(); it != pc.end(); it++){
		scan.push_back(it->x, it->y, it->z);
	}

	// grab the sensor origin
        octomap::point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);

        // scan: const PointCloud&
	// sensor_origin: const octomap::point3d&
        m_octree->insertPointCloud( scan, sensorOrigin);

        //if (m_compressMap) {
        //    m_octree->prune();
        //}

    }


    void OctomapServer::publishAllNow(){
        rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>();
	publishAll(clock->now());
    }

    void OctomapServer::publishAll(
        const rclcpp::Time &rostime) {

        // ros::WallTime startTime = ros::WallTime::now();
        
        size_t octomap_size = m_octree->size();
        if (octomap_size <= 1) {
            RCLCPP_WARN(
                this->get_logger(),
                "Nothing to publish, octree is empty");
            return;
        }

	if (m_publishMarkerArray || m_publishFreeSpace){
	    publishMarkerArrayOctomap(rostime);
	}	    

	// publish pointcloud with only obstacles
	if (m_publishOccupiedPCL) {
	
            // init pointcloud:
            pcl::PointCloud<PCLPoint> pclCloud;

            // now, traverse all leafs in the tree:
            //for (auto it = m_octree->begin(m_maxTreeDepth),
            //          end = m_octree->end(); it != end; ++it) {
            for (auto it = m_octree->begin_leafs(), end=m_octree->end_leafs(); it!= end; ++it){
            	if (m_octree->isNodeOccupied(*it)) {
	    	 pclCloud.push_back(PCLPoint ( it.getX(), it.getY(), it.getZ() ) );
	         }
	    }
	    // convert to ros msg
	    sensor_msgs::msg::PointCloud2 output;
	    pcl::toROSMsg<PCLPoint>(pclCloud, output);

	    output.header.stamp = rostime;
	    output.header.frame_id = m_worldFrameId;

	    m_occupiedPCLPub -> publish(output);	
	}

        // finish pointcloud:
        if (m_publishBinaryMap) {
            publishBinaryOctoMap(rostime);
        }
        
        if (m_publishFullMap) {
            publishFullOctoMap(rostime);
        }

        /*
        double total_elapsed = (ros::WallTime::now() - startTime).toSec();
        ROS_DEBUG("Map publishing in OctomapServer took %f sec", total_elapsed);
        */
    }

    bool OctomapServer::octomapBinarySrv(
        const std::shared_ptr<OctomapSrv::Request> req,
        std::shared_ptr<OctomapSrv::Response> res) 
   {
        // ros::WallTime startTime = ros::WallTime::now();
        RCLCPP_INFO(this->get_logger(),
                    "Sending binary map data on service request");
        
	res->map.header.frame_id = m_worldFrameId;
        res->map.header.stamp = this->get_clock()->now();
        
	if (!octomap_msgs::binaryMapToMsg(*m_octree, res->map)) {
            return false;
        }
        
        /*
        double total_elapsed = (ros::WallTime::now() - startTime).toSec();
        ROS_INFO("Binary octomap sent in %f sec", total_elapsed);
        */
        return true;
    }

    bool OctomapServer::octomapFullSrv(
        const std::shared_ptr<OctomapSrv::Request> req,
        std::shared_ptr<OctomapSrv::Response> res) 
    {
        RCLCPP_INFO(this->get_logger(),
                    "Sending full map data on service request");
        
	res->map.header.frame_id = m_worldFrameId;
        res->map.header.stamp = this->get_clock()->now();

        if (!octomap_msgs::fullMapToMsg(*m_octree, res->map)) {
            return false;            
        }
        
	return true;
    }


    bool OctomapServer::resetSrv(
        const std::shared_ptr<std_srvs::srv::Empty::Request> req,
        std::shared_ptr<std_srvs::srv::Empty::Response> resp) {
        visualization_msgs::msg::MarkerArray occupiedNodesVis;
        occupiedNodesVis.markers.resize(m_treeDepth + 1);
        rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>();
        auto rostime = clock->now();
        
        m_octree->clear();

        RCLCPP_INFO(this->get_logger(), "Cleared octomap");
        publishAll(rostime);

        return true;
    }

    void OctomapServer::publishBinaryOctoMap(
        const rclcpp::Time& rostime) const{

        octomap_msgs::msg::Octomap map;
        map.header.frame_id = m_worldFrameId;
        map.header.stamp = rostime;

        if (octomap_msgs::binaryMapToMsg(*m_octree, map)) {
            m_binaryMapPub->publish(map);
        } else {
            RCLCPP_ERROR(this->get_logger(),
                         "Error serializing OctoMap");
        }
    }

    void OctomapServer::publishFullOctoMap(
        const rclcpp::Time& rostime) const{

        octomap_msgs::msg::Octomap map;
        map.header.frame_id = m_worldFrameId;
        map.header.stamp = rostime;

        if (octomap_msgs::fullMapToMsg(*m_octree, map)) {
            m_fullMapPub->publish(map);
        } else {            
            RCLCPP_ERROR(this->get_logger(),
                         "Error serializing OctoMap");
        }
    }



    void OctomapServer::publishMarkerArrayOctomap(
	const rclcpp::Time& rostime) const {

        // init markers for free space:
        visualization_msgs::msg::MarkerArray freeNodesVis;
        visualization_msgs::msg::MarkerArray occupiedNodesVis;

	// determine the traversal depth
	int max_depth = 0;
	for (int i=0; i < m_treeDepth+1; i++){
	  if (m_octree->getNodeSize(i) > m_publishResolution){
		  max_depth = i;
	  }
	  else { break; }
	}

	RCLCPP_INFO(this->get_logger(), "max_depth: %d, treeDepth: %d", max_depth, m_treeDepth);
	
	// each array stores all cubes of a different size, one for each depth level:
        freeNodesVis.markers.resize(max_depth+1);
        occupiedNodesVis.markers.resize(max_depth+1);
        
	
	tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, 0.0);        
        geometry_msgs::msg::Pose pose;
        pose.orientation = tf2::toMsg(quaternion);

        // now, traverse all leafs in the tree:
        for (auto it = m_octree->begin(max_depth),
                 end = m_octree->end(); it != end; ++it) {

            if (m_octree->isNodeOccupied(*it) && m_publishMarkerArray) {
                double z = it.getZ();
                double half_size = it.getSize() / 2.0;
                if (z + half_size > m_occupancyMinZ &&
                    z - half_size < m_occupancyMaxZ) {
                    double x = it.getX();
                    double y = it.getY();
                   
		    //create marker:
                   unsigned idx = it.getDepth();
                   assert(idx < occupiedNodesVis.markers.size());

                   geometry_msgs::msg::Point cubeCenter;
                   cubeCenter.x = x;
                   cubeCenter.y = y;
                   cubeCenter.z = z;

                    occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
                    
		    if (m_useHeightMap){
			double minZ = m_pointcloudMinZ;
			double maxZ = m_pointcloudMaxZ;
                        double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
                        occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
                    }
                }
            } 
	    if (m_publishFreeSpace && !m_octree->isNodeOccupied(*it) )  {
                // node not occupied => mark as free in 2D map if unknown so far
                double z = it.getZ();
                double half_size = it.getSize() / 2.0;
                if (z + half_size > m_occupancyMinZ &&
                    z - half_size < m_occupancyMaxZ) {

                    if (m_publishFreeSpace) {
                        double x = it.getX();
                        double y = it.getY();

                        //create marker for free space:
                        if (m_publishFreeSpace) {
                            unsigned idx = it.getDepth();
                            assert(idx < freeNodesVis.markers.size());

                            geometry_msgs::msg::Point cubeCenter;
                            cubeCenter.x = x;
                            cubeCenter.y = y;
                            cubeCenter.z = z;

                            freeNodesVis.markers[idx].points.push_back(cubeCenter);
                        }
                    }
                }
            }
        }

        // finish MarkerArray:
        if (m_publishMarkerArray) {
            for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
                double size = m_octree->getNodeSize(i);

                occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
                occupiedNodesVis.markers[i].header.stamp = rostime;
                occupiedNodesVis.markers[i].ns = "map";
                occupiedNodesVis.markers[i].id = i;
                occupiedNodesVis.markers[i].type =
                    visualization_msgs::msg::Marker::CUBE_LIST;
                occupiedNodesVis.markers[i].scale.x = size;
                occupiedNodesVis.markers[i].scale.y = size;
                occupiedNodesVis.markers[i].scale.z = size;
                occupiedNodesVis.markers[i].color = m_color;


                if (occupiedNodesVis.markers[i].points.size() > 0)
                    occupiedNodesVis.markers[i].action =
                        visualization_msgs::msg::Marker::ADD;
                else
                    occupiedNodesVis.markers[i].action =
                        visualization_msgs::msg::Marker::DELETE;
            }
            m_markerPub->publish(occupiedNodesVis);
        }

        // finish FreeMarkerArray:
        if (m_publishFreeSpace) {
            for (unsigned i= 0; i < freeNodesVis.markers.size(); ++i){
                double size = m_octree->getNodeSize(i);

                freeNodesVis.markers[i].header.frame_id = m_worldFrameId;
                freeNodesVis.markers[i].header.stamp = rostime;
                freeNodesVis.markers[i].ns = "map";
                freeNodesVis.markers[i].id = i;
                freeNodesVis.markers[i].type =
                    visualization_msgs::msg::Marker::CUBE_LIST;
                freeNodesVis.markers[i].scale.x = size;
                freeNodesVis.markers[i].scale.y = size;
                freeNodesVis.markers[i].scale.z = size;
                freeNodesVis.markers[i].color = m_colorFree;

                if (freeNodesVis.markers[i].points.size() > 0)
                    freeNodesVis.markers[i].action =
                        visualization_msgs::msg::Marker::ADD;
                else
                    freeNodesVis.markers[i].action =
                        visualization_msgs::msg::Marker::DELETE;
            }
            m_fmarkerPub->publish(freeNodesVis);
	}

    }
	
    std_msgs::msg::ColorRGBA OctomapServer::heightMapColor(double h) {
        std_msgs::msg::ColorRGBA color;
        color.a = 1.0;
        // blend over HSV-values (more colors)

        double s = 1.0;
        double v = 1.0;

        h -= floor(h);
        h *= 6;
        int i;
        double m, n, f;

        i = floor(h);
        f = h - i;
        if (!(i & 1))
            f = 1 - f; // if i is even
        m = v * (1 - s);
        n = v * (1 - s * f);

        switch (i) {
        case 6:
        case 0:
            color.r = v; color.g = n; color.b = m;
            break;
        case 1:
            color.r = n; color.g = v; color.b = m;
            break;
        case 2:
            color.r = m; color.g = v; color.b = n;
            break;
        case 3:
            color.r = m; color.g = n; color.b = v;
            break;
        case 4:
            color.r = n; color.g = m; color.b = v;
            break;
        case 5:
            color.r = v; color.g = m; color.b = n;
            break;
        default:
            color.r = 1; color.g = 0.5; color.b = 0.5;
            break;
        }
        return color;
    }

    

} // namespace octomap_server

// RCLCPP_COMPONENTS_REGISTER_NODE(octomap_server::OctomapServer)


namespace octomap_publisher {


class OctomapPublisher: public rclcpp::Node
{
  public:
    OctomapPublisher(std::shared_ptr<OcTreeT> octree_ptr)
      : Node("octomap_publisher"),
	m_octree(octree_ptr)
    {
	 pub_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
	 timer_ = this->create_wall_timer( std::chrono::milliseconds(500), std::bind(&OctomapPublisher::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto msg = std_msgs::msg::String();
      msg.data = "Hello World!";
      pub_ -> publish(msg);
      RCLCPP_INFO(this->get_logger(), "pub: octree_size: %zu", m_octree->size());
    }


    std::shared_ptr<OcTreeT> m_octree;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};


} // namespace octomap_publisher


int main(int argc, char* argv[]){

  rclcpp::init(argc, argv);

  std::cout << "here" << std::endl;

  //rclcpp::Node::SharedPtr node = std::make_shared<octomap_server::OctomapServer>();
  auto node = std::make_shared<octomap_server::OctomapServer>();
  
  rclcpp::Node::SharedPtr pubNode = std::make_shared<octomap_publisher::OctomapPublisher>(node->get_octree());

  rclcpp::executors::MultiThreadedExecutor(executor);
  executor.add_node(node);
  executor.add_node(pubNode);
  
  executor.spin();

  
  rclcpp::shutdown();
  return 0;

}
