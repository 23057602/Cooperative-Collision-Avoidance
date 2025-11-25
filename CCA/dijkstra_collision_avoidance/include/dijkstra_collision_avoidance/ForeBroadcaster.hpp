#pragma once
//ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
//Proprietary includes
#include <cca_interfaces/msg/drive_forecast.hpp>
//standard c++ includes
#include <string>
#include <map>
#include <memory>

class Predictable
{
    public:
        virtual void initialize() = 0;
        virtual double getModelPeriod() = 0;
        virtual void runStepNext() = 0;
        virtual rclcpp::Time getNextTime() = 0;
        virtual geometry_msgs::msg::PoseWithCovariance getNextPose() = 0;
        virtual geometry_msgs::msg::TwistWithCovariance getNextTwist() = 0;
};


class ForeBroadcaster: virtual public rclcpp::Node
{
    protected:
        rclcpp::Publisher<cca_interfaces::msg::DriveForecast>::SharedPtr trajectory_publisher;
        rclcpp::Subscription<cca_interfaces::msg::DriveForecast>::SharedPtr trajectory_subscriber;
        rclcpp::TimerBase::SharedPtr trajectory_publish_timer;
        std::shared_ptr<Predictable> motion_model;
        cca_interfaces::msg::DriveForecast::SharedPtr trajectory;
        std::map<std::string, cca_interfaces::msg::DriveForecast::SharedPtr> trajectory_table;//id acs tbl 2 shd_ptr 4 r/w ctrl
        std::string vehicle_id;
        double publish_frequency;
        std::string fixed_frame;
        std::string robot_frame;
        double conflict_rad;
        double t_pred;
        double separation_dist;

    public:
        ForeBroadcaster(std::string node_name);

        ~ForeBroadcaster();

        void get_trajectory(cca_interfaces::msg::DriveForecast::SharedPtr forecast_msg);

        void run_trajectory();
};