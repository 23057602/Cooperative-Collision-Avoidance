//ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

//proprietary includes
#include <cca_interfaces/srv/global_plan.hpp>
#include <cca_interfaces/msg/cubic_spline.hpp>
#include <cca_utilities/Map_subscriber.hpp>
#include <cca_utilities/Voronoi_graph.hpp>
#include <cca_utilities/Search.hpp>

//standard c++ includes
#include <iostream>
#include <memory>
#include <string>
#include <vector>

class GlobalPlanner : public Map_subscriber
{
public:
    explicit GlobalPlanner() :  Node("Global"), Map_subscriber("Global")//constructor
    {   
        RCLCPP_INFO(this->get_logger(), "The Global Planner has started. Initialising...");
        this->self_srv_ptr = this->create_service<cca_interfaces::srv::GlobalPlan>("Global/get_plan", std::bind(&GlobalPlanner::service_callback, this, std::placeholders::_1, std::placeholders::_2));
        
    }

private://Node objects
    rclcpp::Service<cca_interfaces::srv::GlobalPlan>::SharedPtr self_srv_ptr;
    
    //Service logic
    void service_callback(std::shared_ptr<cca_interfaces::srv::GlobalPlan::Request> request, std::shared_ptr<cca_interfaces::srv::GlobalPlan::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Global plan to [ %3.2g, %3.2g, %3.2g ] from [ %3.2g, %3.2g, %3.2g ] was requested. Searching...",request->end.x,request->end.y,request->end.z,request->start.x,request->start.y,request->start.z);
        auto plan = Search::A_star<std::vector<double>,double>(std::vector<double>({request->start.x,request->start.y,request->start.z}),std::vector<double>({request->end.x,request->end.y,request->end.z}),std::make_unique<Voronoi_graph>(this->conflict_radius,std::vector<double>({request->end.x,request->end.y,request->end.z}),this->map_tree).get());
        if (plan.second.size() < 2){
            RCLCPP_ERROR(this->get_logger(), "No plan could be found.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Route found: %3.2gm travelled over %lu points.", plan.first, plan.second.size());
        if (plan.second.size() < 4){//resample if too few elements
            RCLCPP_INFO(this->get_logger(), "Resampled...");
            for (std::size_t it = 1; it < plan.second.size(); it+=3){
                plan.second.emplace(plan.second.begin() + it, std::vector<double>({2.0*(plan.second[it][0] - plan.second[it-1][0])/3.0 + plan.second[it-1][0], 2.0*(plan.second[it][1] - plan.second[it-1][1])/3.0 + plan.second[it-1][1]}));
                plan.second.emplace(plan.second.begin() + it, std::vector<double>({(plan.second[it][0] - plan.second[it-1][0])/3.0 + plan.second[it-1][0], (plan.second[it][1] - plan.second[it-1][1])/3.0 + plan.second[it-1][1]}));
            }
        }
        RCLCPP_INFO(this->get_logger(), "Packing...");
        for (std::size_t i = 0; i < plan.second.size(); i++){
            geometry_msgs::msg::Point p;
            p.x = plan.second[i][0];
            p.y = plan.second[i][1];
            p.z = plan.second[i][2];
            response->path.push_back(p);
        }
        RCLCPP_INFO(this->get_logger(), "Result has been sent.");
    }
};

//Main
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}