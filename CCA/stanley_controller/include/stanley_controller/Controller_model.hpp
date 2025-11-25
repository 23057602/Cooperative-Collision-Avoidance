#pragma once
//ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
//Proprietary includes
#include <stanley_controller/ForeBroadcaster.hpp>
#include <spline_works/spline_works.hpp>
//standard c++ includes
#include <vector>
#include <cmath>
#include <algorithm>
#include <utility>

using motion_ref = std::vector<std::pair<std::pair<Spline,double>,std::pair<rclcpp::Time,rclcpp::Time>>>;

class Conductor_model: public Predictable
{
    private:
        double frequency;
        double K_ct;
        double K_v;
        double K_it;
        double l_wb;
        double steer_limit;
        double v_hi;
        double v_lo;
        motion_ref g_ref_list;
        motion_ref l_ref_list;
        rclcpp::Time initialTime;
        int64_t init_g_ref_ptr;
        int64_t init_l_ref_ptr;
        geometry_msgs::msg::PoseWithCovariance initial_pose;
        geometry_msgs::msg::TwistWithCovariance initial_twist;
        rclcpp::Time currentTime;
        int64_t current_g_ref_ptr;
        int64_t current_l_ref_ptr;
        geometry_msgs::msg::PoseWithCovariance current_pose;
        geometry_msgs::msg::TwistWithCovariance current_twist;
    public:
        Conductor_model(double freq, double Kt, double Kv, double Ks, double base, double st_lim, double v_max, double v_min);

        ~Conductor_model();

        bool operator<(const Conductor_model& rhs) const;

        void setStack(motion_ref g_ref_stk, motion_ref l_ref_stk = motion_ref());

        double getBase();

        double getSteerLim();

        std::pair<int64_t,int64_t> getRefPtr();

        std::pair<motion_ref,motion_ref> getStack();

        void setInit(double x, double y, double yaw, int64_t ptr, rclcpp::Time t, int64_t l_ptr = -1);

        void setInit(geometry_msgs::msg::Pose pose, int64_t ptr, rclcpp::Time t, int64_t l_ptr = -1);

        void initialize();

        double getModelPeriod();

        void runStepNext();

        rclcpp::Time getNextTime() const;

        rclcpp::Time getNextTime();

        geometry_msgs::msg::PoseWithCovariance getNextPose();

        geometry_msgs::msg::TwistWithCovariance getNextTwist();
};
