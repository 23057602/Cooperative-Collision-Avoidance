#pragma once
#include <vector>
#include <limits>
#include <string>
#include <map>
#include <utility>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/octree/octree_search.h>
#include <cca_interfaces/msg/drive_forecast.hpp>
#include <dijkstra_collision_avoidance/Search.hpp>
#include <stanley_controller/Controller_model.hpp>

using graphNode = std::pair<rclcpp::Time, std::pair<std::vector<double>, std::pair<double, std::vector<double>>>>;//node type = <time, <pose, ref>>

class Fate_graph: public Searchable<graphNode, double>
{
    private:
        rclcpp::Time t_res;
        double steer;
        std::string my_id;
        Conductor_model vehicle;
        std::vector<std::pair<rclcpp::Time, std::pair<double,double>>> cost_ref;
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr static_obstacles;
        std::map<std::string, cca_interfaces::msg::DriveForecast::SharedPtr> dynamic_obstacles;
        double step_time;
        
    public:
        Fate_graph(rclcpp::Time t, double turn, std::string id, Conductor_model model, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr world, std::map<std::string, cca_interfaces::msg::DriveForecast::SharedPtr> bodies, double mano_time = 1.0);
        Fate_graph(std::string id, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr world, std::map<std::string, cca_interfaces::msg::DriveForecast::SharedPtr> bodies);
        ~Fate_graph();
        std::vector<std::pair<graphNode, double>> edgesFrom(graphNode node);
        double getNul();
        double getHzn();
        double callHrst(graphNode node, graphNode end);
        bool spline_conflict(Spline s, double t0, double t1, rclcpp::Time stamp, rclcpp::Duration man_time);
        bool static_Conflict(std::vector<double> pose);
        bool dynamic_Conflict(std::vector<double> pose, rclcpp::Time time);
        std::pair<nav_msgs::msg::Odometry,nav_msgs::msg::Odometry> get_forecast_bounds(rclcpp::Time t, std::vector<nav_msgs::msg::Odometry> p);
        std::pair<double, double> cost_ref_search(rclcpp::Time time);
        bool stop(graphNode &g);
};