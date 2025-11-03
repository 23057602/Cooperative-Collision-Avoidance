//ROS2 includes
#include <rclcpp/rclcpp.hpp>

//proprietary includes
#include <cca_utilities/Map_subscriber.hpp>
#include <cca_utilities/ForeBroadcaster.hpp>
#include <cca_utilities/Conductor_model.hpp>
#include <cca_interfaces/msg/ref_seq.hpp>
#include <cca_interfaces/action/drive.hpp>
#include <cca_interfaces/msg/over_ref_seq.hpp>
#include <cca_interfaces/srv/stop_prediction.hpp>
#include <cca_utilities/Spline_works.hpp>
#include <cca_utilities/Fate_graph.hpp>
//standard c++ includes
#include <set>
#include <ctime>
#include <random>

enum class Cooperative_State : char {SEARCH,WAIT,BID,QUEUE,PLAN,HOLD,RESIGN};

class LocalPlanner: public Map_subscriber, public ForeBroadcaster
{
    public:
        explicit LocalPlanner(): Node("Local"), Map_subscriber("Local"), ForeBroadcaster("Local")
        {
            RCLCPP_INFO(this->get_logger(), "The Local Planner has started. Initialising...");
            //get parameters
            this->declare_parameter("resolution_horizon", 1.0);
            this->get_parameter("resolution_horizon", this->resolution_time);
            this->declare_parameter("evasive_turn_angle", 0.7071);
            this->get_parameter("evasive_turn_angle", this->swerve);
            this->declare_parameter("replan_timeout", 1.0);
            this->get_parameter("replan_timeout", this->timeout);
            this->declare_parameter("planning_margin", 1.0);
            this->get_parameter("planning_margin", this->margin);
            this->declare_parameter("evasive_step_time", 1.0);
            this->get_parameter("evasive_step_time", this->step_size);
            //vehicle model parameters
            double controller_frequency;
            double controller_Kt;
            double controller_Kv;
            double controller_Ks;
            double vehicle_wheel_base;
            double vehicle_steering_limit;
            double fast;
            double slow;
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
            this->get_parameter("evasive_fast_speed", fast);
            this->declare_parameter("evasive_slow_speed", 0.5);
            this->get_parameter("evasive_slow_speed", slow);
            //setup
            this->mission_subscriber = this->create_subscription<cca_interfaces::msg::RefSeq>("plan", 10, std::bind(&LocalPlanner::get_plan, this, std::placeholders::_1));
            this->current_ref_subscriber = this->create_subscription<cca_interfaces::action::Drive_FeedbackMessage>("_action/feedback", 10, std::bind(&LocalPlanner::get_ref_ptr, this, std::placeholders::_1));
            this->override_publisher = this->create_publisher<cca_interfaces::msg::OverRefSeq>("resolution", 10);
            this->trajectory_subscriber = this->create_subscription<cca_interfaces::msg::DriveForecast>("/predicted_trajectory", 10, std::bind(&LocalPlanner::detect, this, std::placeholders::_1));
            this->motion_model = std::dynamic_pointer_cast<Predictable>(std::make_shared<Conductor_model>(controller_frequency, controller_Kt, controller_Kv, controller_Ks, vehicle_wheel_base, vehicle_steering_limit, fast, slow));
            this->toggle_publishing = this->create_client<cca_interfaces::srv::StopPrediction>("toggle_publishing");
        }

    private:
        rclcpp::Subscription<cca_interfaces::msg::RefSeq>::SharedPtr mission_subscriber;
        rclcpp::Subscription<cca_interfaces::action::Drive_FeedbackMessage>::SharedPtr current_ref_subscriber;
        rclcpp::Publisher<cca_interfaces::msg::OverRefSeq>::SharedPtr override_publisher;
        rclcpp::Client<cca_interfaces::srv::StopPrediction>::SharedPtr toggle_publishing;
        double resolution_time;
        double swerve;
        double timeout;
        double margin;
        double step_size;

        void detect(cca_interfaces::msg::DriveForecast::SharedPtr forecast_msg)
        {
            static Cooperative_State current_state = Cooperative_State::SEARCH;
            static rclcpp::Time last_replan = this->get_clock()->now() - rclcpp::Duration::from_seconds(this->timeout) * 2.0;
            if((forecast_msg->vehicle_id == this->vehicle_id) && (this->trajectory_table.find(this->vehicle_id) != this->trajectory_table.end())){//preserve group and priority
                forecast_msg->group_id = this->trajectory_table[this->vehicle_id]->group_id;
                forecast_msg->priority = this->trajectory_table[this->vehicle_id]->priority;
                }
            this->trajectory_table[forecast_msg->vehicle_id] = forecast_msg;//create or reassign dynamic obstacle table row
            if(this->trajectory_table.find(this->vehicle_id) == this->trajectory_table.end()){return;}//no conflict info
            auto vhc = std::dynamic_pointer_cast<Conductor_model>(this->motion_model);
            if(vhc->getStack().first.size() == 0){return;}//no plan received
            switch (current_state)
            {
                case Cooperative_State::SEARCH:
                    {
                        if(rclcpp::Time(forecast_msg->header.stamp) < last_replan + rclcpp::Duration::from_seconds(this->timeout)){return;}//callback overrun protection
                        Fate_graph col_sim(this->vehicle_id,this->map_tree,this->trajectory_table);//conflict detection object
                        std::set<std::string> conflict_list;
                        for(auto data: this->trajectory_table[this->vehicle_id]->trajectory){//conflict check
                            std::vector<double> p ({data.pose.pose.position.x, data.pose.pose.position.y});
                            if(col_sim.static_Conflict(p)){conflict_list.insert("");}//check static obstacle
                            for(auto car: this->trajectory_table){//check dynamic obstacle
                                if(car.first == this->vehicle_id){continue;}
                                auto bounds = col_sim.get_forecast_bounds(data.header.stamp, car.second->trajectory);
                                //find t parameter
                                double t = 0.0;
                                if(rclcpp::Time(bounds.second.header.stamp) > rclcpp::Time(bounds.first.header.stamp)){
                                    t = std::clamp((rclcpp::Time(data.header.stamp).seconds() - rclcpp::Time(bounds.first.header.stamp).seconds())/(rclcpp::Time(bounds.second.header.stamp) - rclcpp::Time(bounds.first.header.stamp)).seconds(), 0.0, 1.0);
                                }
                                //interpolate poses
                                std::vector<double> int_pnt({t*(bounds.second.pose.pose.position.x - bounds.first.pose.pose.position.x) + bounds.first.pose.pose.position.x,
                                                            t*(bounds.second.pose.pose.position.y - bounds.first.pose.pose.position.y) + bounds.first.pose.pose.position.y});
                                //conflict check the interpolation
                                double distance = std::sqrt(std::max((p[0] - int_pnt[0])*(p[0] - int_pnt[0]) + (p[1] - int_pnt[1])*(p[1] - int_pnt[1]), 0.0));
                                if(distance < (this->trajectory_table[this->vehicle_id]->conflict_radius + car.second->conflict_radius + std::max(this->trajectory_table[this->vehicle_id]->separation,car.second->separation))){
                                    conflict_list.insert(car.first);
                                }
                            }
                        }
                        if(forecast_msg->group_id.find(this->vehicle_id) != std::string::npos){//check and process any notification
                            auto msg_conflict_list = this->get_group_members(forecast_msg->group_id);
                            conflict_list.insert(msg_conflict_list.begin(),msg_conflict_list.end());
                            conflict_list.erase(this->vehicle_id);
                        }
                        if(conflict_list.size() * conflict_list.count("") == 1){//only a static obstacle detected
                            RCLCPP_INFO(this->get_logger(), "A static conflict was detected. Planning resolution...");
                            current_state = Cooperative_State::PLAN;//recurrsive call to conflict reolution
                            this->detect(forecast_msg);
                            current_state = Cooperative_State::SEARCH;//revert state
                        } else if(conflict_list.size() > 0){//dynamic obstacle
                            conflict_list.erase("");
                            RCLCPP_INFO(this->get_logger(), "A conflict with %lu object(s) was detected. Starting resolution...", conflict_list.size());
                            this->toggle_publishing->async_send_request(std::make_shared<cca_interfaces::srv::StopPrediction::Request>());//TO-DO: prediction stopping service call
                            //this->trajectory_table[this->vehicle_id]->group_id = this->vehicle_id;//construct group
                            //for(auto id: conflict_list){this->trajectory_table[this->vehicle_id]->group_id += ("," + id);}
                            int64_t i = 0;
                            for(auto row = this->trajectory_table.begin(); row != this->trajectory_table.end(); row++){//involve all vehicles
                                this->trajectory_table[this->vehicle_id]->group_id += (row->first + ",");
                            }
                            this->trajectory_table[this->vehicle_id]->group_id.pop_back();
                            this->trajectory_publisher->publish(*(this->trajectory_table[this->vehicle_id]));//call or acknowledge group
                            for(auto row = this->trajectory_table.begin(); row != this->trajectory_table.end(); row++){//assign priorities to all vehicles
                                row->second->priority = i;
                                i++;
                            }
                            //current_state = Cooperative_State::WAIT;
                            if(this->trajectory_table.begin()->first == this->vehicle_id){
                                current_state = Cooperative_State::PLAN;//recurrsive call to conflict reolution
                                this->detect(forecast_msg);
                            }else{
                                current_state = Cooperative_State::QUEUE;
                                RCLCPP_INFO(this->get_logger(), "Entering queue...");
                            }
                        }
                    }
                    break;
                case Cooperative_State::PLAN:
                    {
                        if(rclcpp::Time(forecast_msg->header.stamp) < last_replan + rclcpp::Duration::from_seconds(this->timeout)){return;}//callback overrun protection
                        RCLCPP_INFO(this->get_logger(), "Planning...");
                        std::size_t time_indx = this->trajectory_table[this->vehicle_id]->trajectory.size();
                        std::size_t i = 0;
                        while(i + 1 < time_indx){//estimate current state
                            if(rclcpp::Time(this->trajectory_table[this->vehicle_id]->trajectory[(i + time_indx)/2].header.stamp) < this->get_clock()->now()){
                                i = (i + time_indx)/2;
                            }else{
                                time_indx = (i + time_indx)/2;
                            }
                        }
                        vhc->initialize();//prepare vehicle model
                        vhc->setInit(this->trajectory_table[this->vehicle_id]->trajectory[time_indx].pose.pose,vhc->getRefPtr().first,rclcpp::Time(this->trajectory_table[this->vehicle_id]->trajectory[time_indx].header.stamp));
                        vhc->initialize();
                        this->trajectory_table[this->vehicle_id]->conflict_radius += this->margin;//inflate conflict area for planning
                        i = 0;
                        for(auto row = this->trajectory_table.begin(); row != this->trajectory_table.end(); row++){//assign priorities to all vehicles
                            row->second->priority = i;
                            i++;
                        }
                        Fate_graph resolving_graph(vhc->getNextTime() + rclcpp::Duration::from_seconds(this->resolution_time),this->swerve,this->vehicle_id,*vhc,this->map_tree,this->trajectory_table,this->step_size);//prepare resolution graph
                        auto pos = vhc->getNextPose();
                        std::vector<double> start_config({pos.pose.position.x,pos.pose.position.y,std::atan2(2 * (pos.pose.orientation.w * pos.pose.orientation.z + pos.pose.orientation.x * pos.pose.orientation.y), 1 - 2 * (pos.pose.orientation.y * pos.pose.orientation.y + pos.pose.orientation.z * pos.pose.orientation.z)),(double) vhc->getRefPtr().first});
                        graphNode start = std::make_pair(vhc->getNextTime(),std::make_pair(start_config,std::make_pair(vhc->getStack().first[(int) start_config[3]].first.second,std::vector<double>({0.0,0.0,start_config[0],std::cos(start_config[2]),0.0,0.0,start_config[1],std::sin(start_config[2])}))));
                        auto stop_cond = [&resolving_graph](graphNode &g)->bool {return resolving_graph.stop(g);};
                        auto itvl = std::clock();
                        auto solution = Search::Dijkstra<graphNode,double,decltype(stop_cond)>(start,&resolving_graph, stop_cond);//plan solution
                        itvl = std::clock() - itvl;
                        this->trajectory_table[this->vehicle_id]->conflict_radius -= this->margin;//revert conflict config
                        cca_interfaces::msg::OverRefSeq ovr;
                        if(solution.second.size() > 1){//check for abort condition
                            RCLCPP_INFO(this->get_logger(), "Resolved in %.3fs with %lu steps. Packing...", (float)(itvl)/CLOCKS_PER_SEC, solution.second.size() - 1);
                            ovr.insert_index = (int64_t) solution.second.rbegin()->second.first[3];
                            for(auto it = solution.second.begin() + 1; it != solution.second.end(); it++){//pack for transmission
                                ovr.ref_stack.seq.emplace_back();
                                ovr.ref_stack.seq.rbegin()->speed = it->second.second.first;
                                ovr.ref_stack.seq.rbegin()->path.mtx_data = it->second.second.second;
                                ovr.ref_stack.seq.rbegin()->start_time = (it-1)->first;
                                ovr.ref_stack.seq.rbegin()->stop_time = it->first;
                            }
                        }
                        this->override_publisher->publish(ovr);
                        RCLCPP_INFO(this->get_logger(), "Resolution Sent.");
                        motion_ref cpp_s_ref({});
                        for(auto step = ovr.ref_stack.seq.rbegin(); step != ovr.ref_stack.seq.rend(); step++){
                            cpp_s_ref.emplace_back(std::make_pair(Spline(Spline_make::hermite_mtx,Spline_make::deserialize(step->path.mtx_data,step->path.ROWS,step->path.cols).block(0,0,4,2).eval()),step->speed),std::make_pair(rclcpp::Time(step->start_time),rclcpp::Time(step->stop_time)));
                        }
                        vhc->initialize();
                        vhc->setStack(vhc->getStack().first,cpp_s_ref);
                        vhc->setInit(vhc->getNextPose().pose, vhc->getRefPtr().first, vhc->getNextTime(), cpp_s_ref.size() - 1);//reinitialise
                        vhc->initialize();
                        this->trajectory_table[this->vehicle_id]->trajectory.clear();
                        for(int64_t i = 0; i < (int64_t)(this->resolution_time/vhc->getModelPeriod()) + 1; i++){
                            vhc->runStepNext();
                            nav_msgs::msg::Odometry prediction;
                            prediction.header.frame_id = this->fixed_frame;
                            prediction.header.stamp = vhc->getNextTime();
                            prediction.child_frame_id = this->robot_frame;
                            prediction.pose = vhc->getNextPose();
                            prediction.twist = vhc->getNextTwist();
                            this->trajectory_table[this->vehicle_id]->trajectory.push_back(prediction);
                        }
                        this->trajectory_table[this->vehicle_id]->header.stamp = this->get_clock()->now();
                        this->trajectory_publisher->publish(*(this->trajectory_table[this->vehicle_id]));
                        vhc->setStack(vhc->getStack().first);
                        vhc->setInit(vhc->getNextPose().pose, vhc->getRefPtr().first, vhc->getNextTime());//reinitialise
                        vhc->initialize();
                        last_replan = this->get_clock()->now();
                        current_state = Cooperative_State::HOLD;
                    }
                    break;
                case Cooperative_State::WAIT:
                    {
                        auto conflict_list = this->get_group_members(this->trajectory_table[this->vehicle_id]->group_id);
                        bool all_memb_ack = true;
                        for(auto member: conflict_list){
                            auto mem_conflict_list = this->get_group_members(this->trajectory_table[member]->group_id);
                            if(mem_conflict_list == conflict_list){continue;}//acknowledge succuessful
                            all_memb_ack = false;
                            if(mem_conflict_list.find(this->vehicle_id) == mem_conflict_list.end()){continue;}//acknowledge failed
                            mem_conflict_list.insert(conflict_list.begin(),conflict_list.end());//expand group to include differing members
                            this->trajectory_table[this->vehicle_id]->group_id = this->vehicle_id;
                            mem_conflict_list.erase(this->vehicle_id);
                            for(auto new_id: mem_conflict_list){this->trajectory_table[this->vehicle_id]->group_id += ("," + new_id);}
                        }
                        if(all_memb_ack){
                            current_state = Cooperative_State::BID;
                            this->trajectory_table[this->vehicle_id]->priority = this->gen_priority(conflict_list.size() - 1);//first bid
                            this->trajectory_publisher->publish(*(this->trajectory_table[this->vehicle_id]));
                        }
                    }
                    break;
                case Cooperative_State::BID:
                    {
                        auto conflict_list = this->get_group_members(this->trajectory_table[this->vehicle_id]->group_id);
                        std::map<int64_t,std::vector<std::string>> priority_table;//record of priority bidding
                        for(auto member: conflict_list){priority_table[this->trajectory_table[member]->priority].push_back(member);}
                        if(priority_table.find(-1) == priority_table.end()){return;}//check that all bids are in before proceeding
                        std::vector<int64_t> remaining_priorities({});
                        for(auto priority_entry: priority_table){
                            if(priority_entry.second.size() > 1){//check for a clash
                                for(auto id: priority_entry.second){//reject for rebidding
                                    this->trajectory_table[id]->priority = -1;
                                }
                                remaining_priorities.push_back(priority_entry.first);//collect open slots
                            }
                        }
                        if(this->trajectory_table[this->vehicle_id]->priority == -1){//check if bid succeeded
                            this->trajectory_table[this->vehicle_id]->priority = remaining_priorities[this->gen_priority(remaining_priorities.size() - 1)];//reroll from remaining slots
                            this->trajectory_publisher->publish(*(this->trajectory_table[this->vehicle_id]));
                        }
                        if(remaining_priorities.size() == 0){//check that all priorities were indeed uniquely distributed
                            current_state = Cooperative_State::QUEUE;//next state
                            if(this->trajectory_table[this->vehicle_id]->priority == 0){//check if self is first
                                current_state = Cooperative_State::PLAN;//take turn via recurrsive call
                                this->detect(forecast_msg);
                            }
                        }
                    }
                    break;
                case Cooperative_State::QUEUE:
                    {
                        if(this->get_group_members(forecast_msg->group_id) == this->get_group_members(this->trajectory_table[this->vehicle_id]->group_id)){//check membership
                            if((forecast_msg->priority + 1) == this->trajectory_table[this->vehicle_id]->priority){//check if your turn is next
                                RCLCPP_INFO(this->get_logger(), "ROS time message delay of %.3fms.", (this->get_clock()->now() - forecast_msg->header.stamp).seconds()*1e3);
                                current_state = Cooperative_State::PLAN;//take turn via recurrsive call
                                this->detect(forecast_msg);
                            }
                        }
                    }
                    break;
                case Cooperative_State::HOLD:
                    {
                        auto conflict_list = this->get_group_members(this->trajectory_table[this->vehicle_id]->group_id);
                        if(this->get_group_members(forecast_msg->group_id) == conflict_list){//check membership
                            if(forecast_msg->priority == (int64_t)(conflict_list.size() - 1)){//check if last priority has published
                                current_state = Cooperative_State::RESIGN;//act via recurrsive call
                                this->detect(forecast_msg);
                            }
                        }
                    }
                    break;
                case Cooperative_State::RESIGN:
                    {
                        this->trajectory_table[this->vehicle_id]->priority = -1;
                        this->trajectory_table[this->vehicle_id]->group_id.clear();
                        //TO-DO: Restart Scout publisher service
                        auto request = std::make_shared<cca_interfaces::srv::StopPrediction::Request>();
                        request->stop = false;
                        this->toggle_publishing->async_send_request(request);
                        last_replan = this->get_clock()->now();
                        RCLCPP_INFO(this->get_logger(), "Coop complete.");
                        current_state = Cooperative_State::SEARCH;
                    }
                    break;
                default:
                    current_state = Cooperative_State::SEARCH;
                    break;
            }
        }

        std::set<std::string> get_group_members(std::string group_id)
        {
            std::stringstream group(group_id);
            std::string id;
            std::set<std::string> member_list;
            while(std::getline(group, id, ',')){member_list.insert(id);}
            return member_list;
        }

        int64_t gen_priority(int64_t max, int64_t min = 0)
        {
            std::random_device rd; // obtain a random number from hardware
            std::mt19937 gen(rd()); // seed the generator
            std::uniform_int_distribution<int64_t> distr(min, max); // define the range
            return distr(gen);
        }

        //controller model management functions
        void get_plan(cca_interfaces::msg::RefSeq::SharedPtr plan_msg)
        {
            motion_ref cpp_s_ref({});
            for (auto step = plan_msg->seq.rbegin(); step != plan_msg->seq.rend(); step++){
                cpp_s_ref.emplace_back(std::make_pair(Spline(Spline_make::hermite_mtx,Spline_make::deserialize(step->path.mtx_data,step->path.ROWS,step->path.cols).block(0,0,4,2).eval()),step->speed),std::make_pair(rclcpp::Time(step->start_time),rclcpp::Time(step->stop_time)));
            }
            auto new_mod = std::make_shared<Conductor_model>(* std::dynamic_pointer_cast<Conductor_model>(this->motion_model));//cast and copy
            new_mod->initialize();
            new_mod->setStack(cpp_s_ref);
            new_mod->setInit(new_mod->getNextPose().pose, new_mod->getStack().first.size() - 1, new_mod->getNextTime());//reinitialise
            this->motion_model = std::dynamic_pointer_cast<Predictable>(new_mod);//overwrite previous model
            if(plan_msg->seq.size() == 0){
                RCLCPP_INFO(this->get_logger(), "Conflict detection stopped.");
            }else{
                RCLCPP_INFO(this->get_logger(), "A global reference was received. Starting conflict detection...");
            }
        }

        void get_ref_ptr(cca_interfaces::action::Drive_FeedbackMessage::SharedPtr ref_ptr_msg)
        {
            auto new_mod = std::make_shared<Conductor_model>(* std::dynamic_pointer_cast<Conductor_model>(this->motion_model));//cast and copy
            new_mod->initialize();
            new_mod->setInit(new_mod->getNextPose().pose, ref_ptr_msg->feedback.ref_index, new_mod->getNextTime());//reinitialise global only, local plan update disallowed
            new_mod->initialize();
            this->motion_model = std::dynamic_pointer_cast<Predictable>(new_mod);//overwrite previous model
            auto ptrs = new_mod->getRefPtr();
            RCLCPP_INFO(this->get_logger(), "Initial reference updated to: %ld/%ld", ptrs.first,ptrs.second);
        }
};

//Main
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}