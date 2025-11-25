//ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
//Proprietary includes
#include <stanley_controller/ForeBroadcaster.hpp>
#include <stanley_controller/Controller_model.hpp>
#include <spline_works/spline_works.hpp>
//standard c++ includes
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <utility>

Conductor_model::Conductor_model(double freq, double Kt, double Kv, double Ks, double base, double st_lim, double v_max, double v_min): frequency(freq), K_ct(Kt), K_v(Kv), K_it(Ks), l_wb(base), steer_limit(st_lim), v_hi(v_max), v_lo(v_min)
{
    this->g_ref_list.clear();
    this->l_ref_list.clear();
    this->init_g_ref_ptr = -1;
    this->init_l_ref_ptr = -1;
    this->initialTime = rclcpp::Time();
    this->initial_pose = geometry_msgs::msg::PoseWithCovariance();
    this->initial_twist = geometry_msgs::msg::TwistWithCovariance();
    this->initialize();
}

Conductor_model::~Conductor_model()
{
}

bool Conductor_model::operator<(const Conductor_model &rhs) const
{
    return this->currentTime < rhs.currentTime;
}

void Conductor_model::setStack(motion_ref g_ref_stk, motion_ref l_ref_stk)
{
    this->g_ref_list = g_ref_stk;
    this->l_ref_list = l_ref_stk;
}

double Conductor_model::getBase()
{
    return this->l_wb;
}

double Conductor_model::getSteerLim()
{
    return this->steer_limit;
}

std::pair<int64_t,int64_t> Conductor_model::getRefPtr()
{
    return std::make_pair(this->current_g_ref_ptr,this->current_l_ref_ptr);
}

std::pair<motion_ref,motion_ref> Conductor_model::getStack()
{
    return std::make_pair(this->g_ref_list,this->l_ref_list);
}

void Conductor_model::setInit(double x, double y, double yaw, int64_t ptr, rclcpp::Time t, int64_t l_ptr)
{
    this->initialTime = t;
    this->init_g_ref_ptr = ptr;
    this->init_l_ref_ptr = l_ptr;
    this->initial_pose.pose.orientation.w = std::cos(0.5 * yaw);
    this->initial_pose.pose.orientation.z = std::sin(0.5 * yaw);
    this->initial_pose.pose.position.x = x;
    this->initial_pose.pose.position.y = y;
}

void Conductor_model::setInit(geometry_msgs::msg::Pose pose, int64_t ptr, rclcpp::Time t, int64_t l_ptr)
{
    this->initialTime = t;
    this->init_g_ref_ptr = ptr;
    this->init_l_ref_ptr = l_ptr;
    this->initial_pose.pose = pose;
}

void Conductor_model::initialize()
{
    this->currentTime = this->initialTime;
    this->current_g_ref_ptr = this->init_g_ref_ptr;
    this->current_l_ref_ptr = this->init_l_ref_ptr;
    this->current_pose = this->initial_pose;
    this->current_twist = this->initial_twist;
}

void Conductor_model::runStepNext()
{
    //reference manage
    if (this->current_g_ref_ptr < 0){
        this->currentTime = this->currentTime + rclcpp::Duration::from_seconds(1.0/this->frequency);
        this->current_twist = geometry_msgs::msg::TwistWithCovariance();
        return;
    }
    int64_t * rf_ptr = &(this->current_l_ref_ptr);
    motion_ref * rf_lst = &(this->l_ref_list);
    if(this->current_l_ref_ptr < 0){
        rf_ptr = &(this->current_g_ref_ptr);
        rf_lst = &(this->g_ref_list);
    }
    
    //current state
    auto car = Eigen::RowVector2d();
    car.row(0) << this->current_pose.pose.position.x, this->current_pose.pose.position.y;
    auto car_yaw = std::atan2(2 * (this->current_pose.pose.orientation.w * this->current_pose.pose.orientation.z + this->current_pose.pose.orientation.x * this->current_pose.pose.orientation.y), 1 - 2 * (this->current_pose.pose.orientation.y * this->current_pose.pose.orientation.y + this->current_pose.pose.orientation.z * this->current_pose.pose.orientation.z));
    
    std::size_t first = 0;
    std::size_t last = this->g_ref_list.size();
    while(first+1 < last){
        if(this->currentTime < this->g_ref_list[(first + last)/2].second.first){
            first = (first + last)/2;
        }else{
            last = (first + last)/2;
        }
    }
    int64_t c_ref_ptr = (first + last)/2;
    //spline
    auto ref_len = this->g_ref_list[c_ref_ptr].first.first.getLength() * std::clamp((this->currentTime - this->g_ref_list[c_ref_ptr].second.first).seconds()/(this->g_ref_list[c_ref_ptr].second.second - this->g_ref_list[c_ref_ptr].second.first).seconds(),0.0,1.0);
    auto tg = this->g_ref_list[this->current_g_ref_ptr].first.first.getClosestT(car);
    auto curr_len = this->g_ref_list[this->current_g_ref_ptr].first.first.getLength(tg);
    double cross_len = 0;
    for(auto i = std::min((int64_t) c_ref_ptr,this->current_g_ref_ptr); i < std::max((int64_t) c_ref_ptr,this->current_g_ref_ptr); i++){cross_len += this->g_ref_list[i].first.first.getLength();}
    auto v = std::clamp((*rf_lst)[*rf_ptr].first.second + this->K_it*(ref_len - curr_len + (2*(c_ref_ptr > this->current_g_ref_ptr) - 1) * cross_len),this->v_lo,this->v_hi);
    
    auto t = (rf_ptr == &(this->current_g_ref_ptr)) ? tg : (*rf_lst)[*rf_ptr].first.first.getClosestT(car);
    auto track_pos = (*rf_lst)[*rf_ptr].first.first(t);
    auto track_tan = (*rf_lst)[*rf_ptr].first.first.getTangent(t);

    //control
    auto track_heading = std::atan2(track_tan(0,1), track_tan(0,0));
    auto pose_cross = cos(car_yaw)*sin(track_heading) - sin(car_yaw)*cos(track_heading);
    auto ctrl_Herr = ((pose_cross >= 0.0) - (pose_cross < 0.0))*std::acos(std::clamp(std::cos(car_yaw)*std::cos(track_heading) + std::sin(car_yaw)*std::sin(track_heading),-1.0,1.0));
    auto ctrl_CTerr = (car(0,0) - track_pos(0,0))*std::sin(track_heading) - (car(0,1) - track_pos(0,1))*std::cos(track_heading);
    double cmd_law = std::clamp(ctrl_Herr + std::atan(this->K_ct * ctrl_CTerr/(v + this->K_v)),-1 * this->steer_limit, this->steer_limit);

    //next state
    this->currentTime = this->currentTime + rclcpp::Duration::from_seconds(1.0/this->frequency);
    this->current_twist.twist.linear.x = v;
    this->current_twist.twist.angular.z = this->current_twist.twist.linear.x * std::tan(cmd_law)/ this->l_wb;
    double new_head = car_yaw + this->current_twist.twist.angular.z/this->frequency;
    this->current_pose.pose.orientation.w = std::cos(0.5 * new_head);
    this->current_pose.pose.orientation.z = std::sin(0.5 * new_head);
    this->current_pose.pose.position.x += this->current_twist.twist.linear.x * std::cos(new_head)/this->frequency;
    this->current_pose.pose.position.y += this->current_twist.twist.linear.x * std::sin(new_head)/this->frequency;
    //reference switch
    if (tg >= 1.0){this->current_g_ref_ptr -= 1;}
    if ((t >= 1.0) && (this->current_l_ref_ptr >= 0)){this->current_l_ref_ptr -= 1;}
}

double Conductor_model::getModelPeriod()
{
    return 1.0/this->frequency;
}

rclcpp::Time Conductor_model::getNextTime() const
{
    return this->currentTime;
}

rclcpp::Time Conductor_model::getNextTime()
{
    return this->currentTime;
}

geometry_msgs::msg::PoseWithCovariance Conductor_model::getNextPose()
{
    return this->current_pose;
}

geometry_msgs::msg::TwistWithCovariance Conductor_model::getNextTwist()
{
    return this->current_twist;
}