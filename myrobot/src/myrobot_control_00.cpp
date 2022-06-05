#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include <chrono>
#include <cstdlib>
#include <memory>
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

const std::string MOVE_GROUP = "myrobot_arm";

class MyRobotControl : public rclcpp::Node
{
    public:
        MyRobotControl(): Node("myrobot_controller")//, move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)),MOVE_GROUP)
        {
            // this->move_group_.setMaxAccelerationScalingFactor(1.0);
            // this->move_group_.setMaxVelocityScalingFactor(1.0);
            // target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose",rclcpp::QoS(1),std::bind(&MyRobotControl::myrobot_move,this,std::placeholders::_1));
            // grasping_sub = this->create_subscription<std_msgs::msg::Bool>("/robot_model/grasping",rclcpp::QoS(1),std::bind(&MyRobotControl::grasping_updater,this,std::placeholders::_1));
            grasping_cli_node = rclcpp::Node::make_shared("grasping_cli_node");
            grasping_cli = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
            RCLCPP_INFO(this->get_logger(),"Initialization successful");
            // RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_.getPlanningFrame().c_str());
            // RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_.getEndEffectorLink().c_str());
            this->gripperOn();
        } 
        // moveit::planning_interface::MoveGroupInterface move_group_;
        // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
        // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr grasping_sub;
        rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr grasping_cli;
        std::shared_ptr<rclcpp::Node> grasping_cli_node;
        geometry_msgs::msg::Pose previous_pose;
        bool grasping;
        
    private:
        // void myrobot_move(geometry_msgs::msg::PoseStamped::SharedPtr msg)     //const...
        // {
            
        //     if(msg->pose == previous_pose)
        //     {
        //         RCLCPP_INFO(this->get_logger(),"The same pose");
        //         return;
        //     }
        //     RCLCPP_INFO(this->get_logger(),"Moving to new pose");           
        //     this->move_group_.setGoalTolerance(0.01);
        //     this->move_group_.setPoseTarget(msg->pose);
        //     this->move_group_.move();
        //     previous_pose=msg->pose;
        // }
        // void grasping_updater(std_msgs::msg::Bool::SharedPtr msg)
        // {
        //     this->grasping = msg->data;
        // }
        void gripperOn()
        {
            RCLCPP_INFO(this->get_logger(),"gripperON method called");
            while(!grasping_cli->wait_for_service(1s))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
            }
            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            request->a = 3;
            request->b = 4;
            auto future = grasping_cli->async_send_request(request);
            
            if(rclcpp::spin_until_future_complete(grasping_cli_node,future,3s) == rclcpp::executor::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Response form gripper service: %s", future.get()->sum);
                
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),"Gripper service failed to response.");
            }
            // auto result = future.get()->success;
            // RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"if success: %S",result);
            

        }
       
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    // rclcpp::spin(std::make_shared<MyRobotControl>());
    auto myrobot_controller = std::make_shared<MyRobotControl>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(myrobot_controller);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}