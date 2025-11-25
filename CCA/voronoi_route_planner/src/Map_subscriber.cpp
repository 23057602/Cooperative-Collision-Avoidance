//Standard includes
#include <string>
#include <vector>
#include <algorithm>
//ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
//Library includes
#include <pcl/point_cloud.h>
#include <pcl/types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/search/search.h>
#include <pcl/octree/octree_search.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
//Proprietary includes
#include <voronoi_route_planner/Map_subscriber.hpp>

Map_subscriber::Map_subscriber(std::string node_name): Node(node_name)
{
    if(!has_parameter("resolution")){declare_parameter("resolution", 1.0);}
    get_parameter("resolution",this->resolution);
    if(!has_parameter("conflict_radius")){declare_parameter("conflict_radius", 1.0);}
    get_parameter("conflict_radius",this->conflict_radius);
    if(!has_parameter("map_height")){declare_parameter("map_height", 0.0);}
    get_parameter("map_height",this->mapLevel);
    if(!has_parameter("floor_height")){declare_parameter("floor_height", 0.0);}
    get_parameter("floor_height",this->floor_height);
    this->map_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>("map", 10, std::bind(&Map_subscriber::mapIn, this, std::placeholders::_1));
    this->map_tree = std::make_shared<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(this->resolution);
    this->map_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}

Map_subscriber::Map_subscriber(std::string node_name, double res, double conflict_radius, double map_z, double floor_z):
Node(node_name), resolution(res), conflict_radius(conflict_radius), mapLevel(map_z), floor_height(floor_z)
{
    this->map_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>("map", 10, std::bind(&Map_subscriber::mapIn, this, std::placeholders::_1));
    this->map_tree = std::make_shared<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(res);
    this->map_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
}

Map_subscriber::~Map_subscriber()
{
}

void Map_subscriber::mapIn(const sensor_msgs::msg::PointCloud2::SharedPtr map_msg)
{
    //msg to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr m(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*map_msg,*m);
    auto floor_filter = std::make_shared<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(this->resolution);
    floor_filter->setInputCloud(m);
    floor_filter->addPointsFromInputCloud();
    //search bounds
    std::vector<double> bounds;
    bounds.resize(6,0.0);
    floor_filter->getBoundingBox(bounds[0],bounds[1],bounds[2],bounds[3],bounds[4],bounds[5]);
    //derive new bounds about mapLevel
    Eigen::Vector3f mn(bounds[0],bounds[1],std::max(std::max(bounds[2],(double) this->floor_height),(double)(this->mapLevel - this->conflict_radius)));
    Eigen::Vector3f mx(bounds[3],bounds[4],std::min(bounds[5],(double)(this->mapLevel + this->conflict_radius)));
    //search in bounds
    pcl::IndicesPtr indx(new std::vector<int>());
    floor_filter->boxSearch(mn,mx, *indx);
    //construct octree
    this->map_tree = std::make_shared<pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>>(this->resolution);//Use new tree to prevent r/w conflicts with dependent processes
    for(std::size_t i = 0; i < indx->size(); i++){(*m)[i].z = 0.0;}//flatten
    this->map_tree->setInputCloud(m, indx);
    this->map_tree->addPointsFromInputCloud();
    //contruct cloud
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> occupiedVoxelList;
    this->map_tree->getOccupiedVoxelCenters(occupiedVoxelList);
    pcl::PointCloud<pcl::PointXYZ>::Ptr envr(new pcl::PointCloud<pcl::PointXYZ>);
    this->map_cloud = envr;
    this->map_cloud->assign(occupiedVoxelList.begin(),occupiedVoxelList.end());
}