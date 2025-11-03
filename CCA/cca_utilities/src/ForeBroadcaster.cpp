//ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
//Proprietary includes
#include <cca_interfaces/msg/drive_forecast.hpp>
#include <cca_utilities/ForeBroadcaster.hpp>
//standard c++ includes
#include <string>
#include <cmath>
#include <vector>
#include <map>

ForeBroadcaster::ForeBroadcaster(std::string node_name): Node(node_name)
{
    if(!has_parameter("vehicle_id")){declare_parameter("vehicle_id", node_name);}
    get_parameter("vehicle_id",this->vehicle_id);
    if(!has_parameter("forecast_frequency")){declare_parameter("forecast_frequency", 1.0);}
    get_parameter("forecast_frequency",this->publish_frequency);
    if(!has_parameter("fixed_frame")){declare_parameter("fixed_frame", "map");}
    get_parameter("fixed_frame",this->fixed_frame);
    if(!has_parameter("robot_frame")){declare_parameter("robot_frame", "base_link");}
    get_parameter("robot_frame",this->robot_frame);
    if(!has_parameter("conflict_radius")){declare_parameter("conflict_radius", 1.0);}
    get_parameter("conflict_radius",this->conflict_rad);
    if(!has_parameter("separation_distance")){declare_parameter("separation_distance", 0.0);}
    get_parameter("separation_distance",this->separation_dist);
    if(!has_parameter("prediction_horizon")){declare_parameter("prediction_horizon", 1.0);}
    get_parameter("prediction_horizon",this->t_pred);
    this->trajectory_publisher = this->create_publisher<cca_interfaces::msg::DriveForecast>("/predicted_trajectory", 10);
    this->trajectory_subscriber = this->create_subscription<cca_interfaces::msg::DriveForecast>("/predicted_trajectory", 10, std::bind(&ForeBroadcaster::get_trajectory, this, std::placeholders::_1));
    this->trajectory_publish_timer = this->create_wall_timer(std::chrono::nanoseconds((int64_t)(1e+9/this->publish_frequency)),std::bind(&ForeBroadcaster::run_trajectory, this));
    this->trajectory_publish_timer->cancel();
    this->trajectory = std::make_shared<cca_interfaces::msg::DriveForecast>();
}

ForeBroadcaster::~ForeBroadcaster()
{
}

void ForeBroadcaster::get_trajectory(cca_interfaces::msg::DriveForecast::SharedPtr forecast_msg)
{
    this->trajectory_table[forecast_msg->vehicle_id] = forecast_msg;//create or reassign
}

void ForeBroadcaster::run_trajectory()
{
    if (this->motion_model == nullptr){return;}//segfault protection
    auto model = this->motion_model;//r/w protect local copy
    cca_interfaces::msg::DriveForecast vehicle_path;
    vehicle_path.conflict_radius = this->conflict_rad;
    vehicle_path.separation = this->separation_dist;
    vehicle_path.header.frame_id = this->fixed_frame;
    vehicle_path.vehicle_id = this->vehicle_id;
    model->initialize();
    for(int64_t i = 0; i < (int64_t)(this->t_pred/model->getModelPeriod()) + 1; i++){
        model->runStepNext();
        nav_msgs::msg::Odometry prediction;
        prediction.header.frame_id = this->fixed_frame;
        prediction.header.stamp = model->getNextTime();
        prediction.child_frame_id = this->robot_frame;
        prediction.pose = model->getNextPose();
        prediction.twist = model->getNextTwist();
        vehicle_path.trajectory.push_back(prediction);
    }
    vehicle_path.header.stamp = this->get_clock()->now();
    this->trajectory = std::make_shared<cca_interfaces::msg::DriveForecast>(vehicle_path);
    this->trajectory_publisher->publish(vehicle_path);
}