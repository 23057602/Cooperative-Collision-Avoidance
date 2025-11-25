#pragma once
//Depends: rclcpp, sensor_msgs, Eigen3, PCL 1.2, pcl_conversions, pcl_ros
//ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
//Library includes
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/octree/octree_search.h>

class Map_subscriber: virtual public rclcpp::Node
{
    protected:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_subscription;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr map_tree;
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;
        float resolution;
        float conflict_radius;
        float mapLevel;
        float floor_height;
    public:
        Map_subscriber(std::string node_name);
        Map_subscriber(std::string node_name, double res, double conflict_radius, double map_z, double floor_z);
        ~Map_subscriber();
        void mapIn(const sensor_msgs::msg::PointCloud2::SharedPtr map_msg);
};