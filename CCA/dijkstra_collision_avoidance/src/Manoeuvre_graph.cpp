#include <cmath>
#include <vector>
#include <limits>
#include <string>
#include <map>
#include <utility>
// #include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/octree/octree_search.h>
#include <cca_interfaces/msg/drive_forecast.hpp>
#include <dijkstra_collision_avoidance/Search.hpp>
#include <stanley_controller/Controller_model.hpp>
#include <dijkstra_collision_avoidance/Manoeuvre_graph.hpp>
#include <spline_works/spline_works.hpp>
#include <ctime>
#include <iostream>

//Full constructor: all functions available
Fate_graph::Fate_graph(rclcpp::Time t, double turn, std::string id, Conductor_model model, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr world, std::map<std::string, cca_interfaces::msg::DriveForecast::SharedPtr> bodies, double mano_time): t_res(t), steer(turn), my_id(id), vehicle(model), static_obstacles(world), dynamic_obstacles(bodies), step_time(mano_time)
{
    std::vector<std::pair<rclcpp::Time, std::pair<double,double>>> full_forecast;
    model.initialize();
    auto time_to_horizon = (t - model.getNextTime()).seconds();
    full_forecast.push_back(std::make_pair(model.getNextTime(),std::make_pair(model.getNextPose().pose.position.x, model.getNextPose().pose.position.y)));
    for (std::size_t i = 0; i < (std::size_t)(time_to_horizon/model.getModelPeriod()) + 1; i++){
        model.runStepNext();
        full_forecast.push_back(std::make_pair(model.getNextTime(),std::make_pair(model.getNextPose().pose.position.x, model.getNextPose().pose.position.y)));
    }
    this->cost_ref = full_forecast;
}

//Limited contructor: only _conflict functions are usable
Fate_graph::Fate_graph(std::string id, pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr world, std::map<std::string, cca_interfaces::msg::DriveForecast::SharedPtr> bodies):my_id(id), vehicle(Conductor_model(1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0)), static_obstacles(world), dynamic_obstacles(bodies)
{}

Fate_graph::~Fate_graph()
{
}

std::vector<std::pair<graphNode, double>> Fate_graph::edgesFrom(graphNode node)
{
    //data structures
    std::vector<std::pair<graphNode, double>> manoeuvres({});
    auto previous_plan = this->vehicle.getStack().first;
    std::vector<std::pair<std::pair<Eigen::Matrix<double, 4, 2>, double>,double>> ref_list({});

    if(node.first >= this->t_res){return manoeuvres;}//horizon reached/leaf condition
    if(node.second.first[3] < 0){return manoeuvres;}//end of global plan
    if(this->dynamic_obstacles.find(this->my_id) == this->dynamic_obstacles.end()){return manoeuvres;}//no conflict info

    //turning manoeuvres
    double turn_radius = previous_plan[(int) node.second.first[3]].first.second * this->step_time / this->steer;
    Eigen::RowVector2d xy_init {node.second.first[0],node.second.first[1]};
    ref_list.push_back(std::make_pair(std::make_pair(Spline_make::herm_arc2d(xy_init, node.second.first[2], turn_radius, this->steer), previous_plan[(int) node.second.first[3]].first.second),this->step_time));
    ref_list.push_back(std::make_pair(std::make_pair(Spline_make::herm_arc2d(xy_init, node.second.first[2], -1 * turn_radius, this->steer), previous_plan[(int) node.second.first[3]].first.second),this->step_time));
    //heading manoeuvres
    Eigen::RowVector2d xy_str {node.second.first[0] + previous_plan[(int) node.second.first[3]].first.second * this->step_time * std::cos(node.second.first[2]), node.second.first[1] + previous_plan[(int) node.second.first[3]].first.second * this->step_time * std::sin(node.second.first[2])};
    ref_list.push_back(std::make_pair(std::make_pair(Spline_make::herm_line(xy_init,xy_str), previous_plan[(int) node.second.first[3]].first.second),this->step_time));
    //resume global
    this->vehicle.setInit(node.second.first[0],node.second.first[1],node.second.first[2],(int64_t) node.second.first[3],node.first);
    this->vehicle.initialize();
    Eigen::MatrixXd path_pts((int)(this->step_time/this->vehicle.getModelPeriod()),2);//gather future position
    path_pts.row(0) << node.second.first[0], node.second.first[1];
    for(Eigen::Index i = 1;i < path_pts.rows(); i++){
        this->vehicle.runStepNext();
        path_pts.row(i) << this->vehicle.getNextPose().pose.position.x, this->vehicle.getNextPose().pose.position.y;
    }
    ref_list.push_back(std::make_pair(std::make_pair(Spline_make::fromPoints(path_pts,Spline_make::hermite_mtx), previous_plan[(int) node.second.first[3]].first.second),this->step_time));
    //validate and package
    for(auto ref: ref_list){
        auto next_t = previous_plan[(int) node.second.first[3]].first.first.getClosestT(ref.first.first.row(2).eval());
        auto current_t = previous_plan[(int) node.second.first[3]].first.first.getClosestT(ref.first.first.row(0).eval());
        if(next_t <= current_t){continue;}//prevent plans going backwards
        bool conflict = false;
        auto m_t = rclcpp::Duration(1,0) * ref.second;
        this->vehicle.setStack(previous_plan,motion_ref({std::make_pair(std::make_pair(Spline(Spline_make::hermite_mtx, ref.first.first),ref.first.second),std::make_pair(node.first,node.first + m_t))}));
        this->vehicle.setInit(node.second.first[0],node.second.first[1],node.second.first[2],(int64_t) node.second.first[3],node.first,0);
        this->vehicle.initialize();
        auto current_pose = this->vehicle.getNextPose();
        while(this->vehicle.getRefPtr().second > -1 && this->vehicle.getRefPtr().first > -1){
            this->vehicle.runStepNext();
            current_pose = this->vehicle.getNextPose();
            std::vector<double> pose_v({current_pose.pose.position.x, current_pose.pose.position.y});
            if(this->static_Conflict(pose_v) || this->dynamic_Conflict(pose_v, this->vehicle.getNextTime())){
                conflict = true;
                break;
            }
        }
        if(conflict){continue;}
        auto cost_ref_loc = this->cost_ref_search(this->vehicle.getNextTime());
        cost_ref_loc.first -= current_pose.pose.position.x;
        cost_ref_loc.second -= current_pose.pose.position.y;
        manoeuvres.push_back(//add node edge
            std::make_pair(
                std::make_pair(//node
                    this->vehicle.getNextTime(), 
                    std::make_pair(
                        std::vector<double>({//pose
                            current_pose.pose.position.x,
                            current_pose.pose.position.y,
                            std::atan2(2 * (current_pose.pose.orientation.w * current_pose.pose.orientation.z + current_pose.pose.orientation.x * current_pose.pose.orientation.y), 1 - 2 * (current_pose.pose.orientation.y * current_pose.pose.orientation.y + current_pose.pose.orientation.z * current_pose.pose.orientation.z)),
                            (double)((int)node.second.first[3] - (bool)(next_t == 1.0))//ref_ptr
                            }),
                        std::make_pair(//reference
                            ref.first.second,//speed
                            Spline_make::serialize(ref.first.first)//spline
                        )
                    )
                ),
                cost_ref_loc.first * cost_ref_loc.first + cost_ref_loc.second * cost_ref_loc.second//cost is squared distance
            )
        );
    }
    this->vehicle.setStack(previous_plan);
    return manoeuvres;
}

double Fate_graph::getNul() {return 0.0;}

double Fate_graph::getHzn() {return (double) std::numeric_limits<double>::infinity();}

double Fate_graph::callHrst(graphNode node, graphNode end) {
    (void) node;
    (void) end;
    return 0.0;
}

bool Fate_graph::spline_conflict(Spline s, double t0, double t1, rclcpp::Time stamp, rclcpp::Duration man_time)
{
    Eigen::RowVectorXd diff = s(t1) - s(t0);
    if(std::sqrt(diff(0)*diff(0) + diff(1)*diff(1)) < this->dynamic_obstacles[this->my_id]->conflict_radius){return false;}
    Eigen::RowVectorXd c = s(0.5*(t1 + t0));
    if(this->static_Conflict(std::vector<double>({c(0), c(1)})) || this->dynamic_Conflict(std::vector<double>({c(0), c(1)}), stamp + (man_time * (0.5*(t1 + t0))))){
        return true;
    }else{
        if(this->spline_conflict(s, t0, 0.5*(t1 + t0), stamp, man_time)){
            return true;
        }else{
            return this->spline_conflict(s, 0.5*(t1 + t0), t1, stamp, man_time);
        }
    }
    return false;
}

bool Fate_graph::static_Conflict(std::vector<double> pose)
{
    if(this->static_obstacles.use_count() == 0){return false;}//map exists condition
    if(this->static_obstacles->getTreeDepth() == 0){return false;}//empty map condition
    if(this->dynamic_obstacles.find(this->my_id) == this->dynamic_obstacles.end()){return false;}//no conflict info
    pcl::Indices ind;
    std::vector<float> fs({});
    auto n = this->static_obstacles->radiusSearch(pcl::PointXYZ(pose[0], pose[1], 0.0),this->dynamic_obstacles[this->my_id]->conflict_radius,ind,fs);
    return n > 0;
}

bool Fate_graph::dynamic_Conflict(std::vector<double> pose, rclcpp::Time time)
{
    if(this->dynamic_obstacles.find(this->my_id) == this->dynamic_obstacles.end()){return false;}//no conflict info
    for(auto dynamic_info: this->dynamic_obstacles){
        if(dynamic_info.first == this->my_id){continue;}
        //skip based on group id
        //skip if lower priority (big number is low priority, so strictly greater than >)
        if (dynamic_info.second->priority > this->dynamic_obstacles[this->my_id]->priority){continue;}
        //find pair to interpolate
        auto bounds = this->get_forecast_bounds(time, dynamic_info.second->trajectory);//recursive look up of pair
        //find t parameter
        double t = 0.0;
        if(rclcpp::Time(bounds.second.header.stamp) > rclcpp::Time(bounds.first.header.stamp)){
            t = std::clamp((time.seconds() - rclcpp::Time(bounds.first.header.stamp).seconds())/(rclcpp::Time(bounds.second.header.stamp) - rclcpp::Time(bounds.first.header.stamp)).seconds(), 0.0, 1.0);
        }
        //interpolate poses
        std::vector<double> int_pnt({t*(bounds.second.pose.pose.position.x - bounds.first.pose.pose.position.x) + bounds.first.pose.pose.position.x,
                                    t*(bounds.second.pose.pose.position.y - bounds.first.pose.pose.position.y) + bounds.first.pose.pose.position.y});
        //conflict check interpolation
        double distance = std::sqrt(std::max((pose[0] - int_pnt[0])*(pose[0] - int_pnt[0]) + (pose[1] - int_pnt[1])*(pose[1] - int_pnt[1]), 0.0));
        if(distance < (this->dynamic_obstacles[this->my_id]->conflict_radius + dynamic_info.second->conflict_radius + std::max(this->dynamic_obstacles[this->my_id]->separation,dynamic_info.second->separation))){return true;}
    }
    return false;
}

std::pair<nav_msgs::msg::Odometry,nav_msgs::msg::Odometry> Fate_graph::get_forecast_bounds(rclcpp::Time t, std::vector<nav_msgs::msg::Odometry> p)
{
    std::size_t first = 0;
    std::size_t second = p.size() - 1;
    while (second > first + 1){//check bounds
        if (t < p[(first + second)/2].header.stamp){//binary search time stamp
            second = (first + second)/2;
        }else{
            first = (first + second)/2;
        }
    }
    return std::make_pair(p[first], p[second]);
}

std::pair<double, double> Fate_graph::cost_ref_search(rclcpp::Time time)
{
    std::size_t first = 0;
    std::size_t second = this->cost_ref.size() - 1;
    if (time < this->cost_ref.begin()->first){
        second = first;
    }
    else if (time > this->cost_ref.rbegin()->first){
        first = second;
    }
    else{
        while (second > first + 1){//check bounds
            if (time < this->cost_ref[(first + second)/2].first){//binary search time stamp
                second = (first + second)/2;
            }else{
                first = (first + second)/2;
            }
        }
    }
    //find t parameter
    double t = 0.0;
    if((rclcpp::Time(this->cost_ref[second].first) - rclcpp::Time(this->cost_ref[first].first)).seconds() > 0.0){
        t = (time - rclcpp::Time(this->cost_ref[first].first)).seconds()/(rclcpp::Time(this->cost_ref[second].first) - rclcpp::Time(this->cost_ref[first].first)).seconds();
    }
    return std::make_pair(t*(this->cost_ref[second].second.first - this->cost_ref[first].second.first) + this->cost_ref[first].second.first, t*(this->cost_ref[second].second.second - this->cost_ref[first].second.second) + this->cost_ref[first].second.second);
}

bool Fate_graph::stop(graphNode &g)
{
    return (g.first > this->t_res) || (g.second.first.at(3) < 0);
}
