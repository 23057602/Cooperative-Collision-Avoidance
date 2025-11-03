//ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
//Proprietary includes
#include <cca_utilities/ForeBroadcaster.hpp>
#include <cca_utilities/Conductor_model.hpp>
#include <cca_interfaces/msg/ref_seq.hpp>
#include <cca_interfaces/action/drive.hpp>
#include <cca_interfaces/msg/over_ref_seq.hpp>
#include <cca_interfaces/srv/stop_prediction.hpp>
#include <cca_utilities/Spline_works.hpp>
//standard c++ includes
#include <functional>
#include <vector>
#include <utility>
//library includes
#include <Eigen/Dense>
class Scout: public ForeBroadcaster
{
private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscriber;
    rclcpp::Subscription<cca_interfaces::msg::RefSeq>::SharedPtr mission_subscriber;
    rclcpp::Subscription<cca_interfaces::action::Drive_FeedbackMessage>::SharedPtr current_ref_subscriber;
    rclcpp::Subscription<cca_interfaces::msg::OverRefSeq>::SharedPtr override_subscriber;
    rclcpp::Service<cca_interfaces::srv::StopPrediction>::SharedPtr prediction_toggle;

    void toggle_publishing(std::shared_ptr<cca_interfaces::srv::StopPrediction::Request> request, std::shared_ptr<cca_interfaces::srv::StopPrediction::Response> response)
    {
        if(request->stop){
            this->trajectory_publish_timer->cancel();
            RCLCPP_INFO(this->get_logger(), "Prediction broadcasting has been paused.");
        }else{
            if(std::dynamic_pointer_cast<Conductor_model>(this->motion_model)->getStack().first.size() > 0){this->trajectory_publish_timer->reset();}
            RCLCPP_INFO(this->get_logger(), "Prediction broadcasting has been resumed.");
        }
        (void) response;
    }

    void get_pose(nav_msgs::msg::Odometry::SharedPtr pose_msg)
    {
        //update model
        auto new_mod = std::make_shared<Conductor_model>(* std::dynamic_pointer_cast<Conductor_model>(this->motion_model));//cast and copy
        new_mod->initialize();
        auto ptrs = new_mod->getRefPtr();
        new_mod->setInit(pose_msg->pose.pose, ptrs.first, pose_msg->header.stamp, ptrs.second);//reinitialise
        this->motion_model = std::dynamic_pointer_cast<Predictable>(new_mod);//overwrite previous model
    }

    void get_plan(cca_interfaces::msg::RefSeq::SharedPtr plan_msg)
    {
        if (plan_msg->seq.size() == 0){
            this->trajectory_publish_timer->cancel();
            std::dynamic_pointer_cast<Conductor_model>(this->motion_model)->getStack().first.clear();
            RCLCPP_INFO(this->get_logger(), "Prediction broadcasting has been stopped.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "A global reference was received. Initialising model...");
        motion_ref cpp_s_ref({});
        for(auto step = plan_msg->seq.rbegin(); step != plan_msg->seq.rend(); step++){
            cpp_s_ref.emplace_back(std::make_pair(Spline(Spline_make::hermite_mtx,Spline_make::deserialize(step->path.mtx_data,step->path.ROWS,step->path.cols).block(0,0,4,2).eval()),step->speed),std::make_pair(rclcpp::Time(step->start_time),rclcpp::Time(step->stop_time)));
        }
        auto new_mod = std::make_shared<Conductor_model>(* std::dynamic_pointer_cast<Conductor_model>(this->motion_model));//cast and copy
        new_mod->initialize();
        new_mod->setStack(cpp_s_ref);
        new_mod->setInit(new_mod->getNextPose().pose, new_mod->getStack().first.size() - 1, new_mod->getNextTime());//reinitialise
        this->motion_model = std::dynamic_pointer_cast<Conductor_model>(new_mod);//overwrite previous model
        RCLCPP_INFO(this->get_logger(), "Running...");
        this->trajectory_publish_timer->reset();//start predictions
        
    }

    void get_ref_ptr(cca_interfaces::action::Drive_FeedbackMessage::SharedPtr ref_ptr_msg)
    {
        auto new_mod = std::make_shared<Conductor_model>(* std::dynamic_pointer_cast<Conductor_model>(this->motion_model));//cast and copy
        new_mod->initialize();
        new_mod->setInit(new_mod->getNextPose().pose, ref_ptr_msg->feedback.ref_index, new_mod->getNextTime(), ref_ptr_msg->feedback.local_index);//reinitialise
        new_mod->initialize();
        this->motion_model = std::dynamic_pointer_cast<Predictable>(new_mod);//overwrite previous model
        auto ptrs = new_mod->getRefPtr();
        RCLCPP_INFO(this->get_logger(), "Initial reference updated to: %ld/%ld", ptrs.first,ptrs.second);
    }

    void get_override(cca_interfaces::msg::OverRefSeq::SharedPtr plan_msg)
    {
        if (plan_msg->ref_stack.seq.size() == 0){
            this->trajectory_publish_timer->cancel();
            std::dynamic_pointer_cast<Conductor_model>(this->motion_model)->getStack().first.clear();
            RCLCPP_INFO(this->get_logger(), "Local plan abort condition received. Prediction broadcasting has been stopped.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Local plan received. Overriding...");
        motion_ref cpp_s_ref({});
        for(auto step = plan_msg->ref_stack.seq.rbegin(); step != plan_msg->ref_stack.seq.rend(); step++){
            cpp_s_ref.emplace_back(std::make_pair(Spline(Spline_make::hermite_mtx,Spline_make::deserialize(step->path.mtx_data,step->path.ROWS,step->path.cols).block(0,0,4,2).eval()),step->speed),std::make_pair(rclcpp::Time(step->start_time),rclcpp::Time(step->stop_time)));
        }
        auto new_mod = std::make_shared<Conductor_model>(* std::dynamic_pointer_cast<Conductor_model>(this->motion_model));//cast and copy
        new_mod->initialize();
        auto previous_plan = new_mod->getStack();
        new_mod->setStack(previous_plan.first,cpp_s_ref);
        auto ptrs = new_mod->getRefPtr();
        new_mod->setInit(new_mod->getNextPose().pose, ptrs.first, new_mod->getNextTime(), cpp_s_ref.size() - 1);//reinitialise
        new_mod->initialize();
        this->motion_model = std::dynamic_pointer_cast<Predictable>(new_mod);//overwrite previous model
        RCLCPP_INFO(this->get_logger(), "Overridden.");
    }

public:
    explicit Scout(): Node("Scout"), ForeBroadcaster("Scout")
    {
        RCLCPP_INFO(this->get_logger(), "The Scout Node has started. Initialising node...");
        double controller_frequency;
        double controller_Kt;
        double controller_Kv;
        double controller_Ks;
        double vehicle_wheel_base;
        double vehicle_steering_limit;
        double v_hi;
        double v_lo;
        this->declare_parameter("controller_frequency", 1.0);
        this->get_parameter("controller_frequency", controller_frequency);
        this->declare_parameter("controller_error_gain", 0.0);
        this->get_parameter("controller_error_gain", controller_Kt);
        this->declare_parameter("controller_speed_gain", 1.0);
        this->get_parameter("controller_speed_gain", controller_Kv);
        this->declare_parameter("controller_follow_gain", 1.0);
        this->get_parameter("controller_follow_gain", controller_Ks);
        this->declare_parameter("vehicle_base", 1.0);
        this->get_parameter("vehicle_base", vehicle_wheel_base);
        this->declare_parameter("vehicle_steering_limit", 0.7854);
        this->get_parameter("vehicle_steering_limit", vehicle_steering_limit);
        this->declare_parameter("evasive_fast_speed", 1.0);
        this->get_parameter("evasive_fast_speed", v_hi);
        this->declare_parameter("evasive_slow_speed", 0.01);
        this->get_parameter("evasive_slow_speed", v_lo);
        this->motion_model = std::dynamic_pointer_cast<Predictable>(std::make_shared<Conductor_model>(controller_frequency, controller_Kt, controller_Kv, controller_Ks, vehicle_wheel_base, vehicle_steering_limit, v_hi, v_lo));
        this->pose_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("pose", 10, std::bind(&Scout::get_pose, this, std::placeholders::_1));
        this->mission_subscriber = this->create_subscription<cca_interfaces::msg::RefSeq>("plan", 10, std::bind(&Scout::get_plan, this, std::placeholders::_1));
        this->current_ref_subscriber = this->create_subscription<cca_interfaces::action::Drive_FeedbackMessage>("_action/feedback", 10, std::bind(&Scout::get_ref_ptr, this, std::placeholders::_1));
        this->override_subscriber = this->create_subscription<cca_interfaces::msg::OverRefSeq>("ovr", 10, std::bind(&Scout::get_override, this, std::placeholders::_1));
        this->prediction_toggle = this->create_service<cca_interfaces::srv::StopPrediction>("toggle_publishing", std::bind(&Scout::toggle_publishing, this, std::placeholders::_1, std::placeholders::_2));
    }
};


//Main
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Scout>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}