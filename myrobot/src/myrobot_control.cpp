#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include "std_srvs/srv/set_bool.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

const std::string MOVE_GROUP = "myrobot_arm";

class MyRobotControl : public rclcpp::Node
{
    public:
        MyRobotControl() : Node("myrobot_control",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)), 
            move_group(std::shared_ptr<rclcpp::Node>(std::move(this)),MOVE_GROUP)
        {
            //items for gripper ON or OFF service
            grasping_cli_node  = rclcpp::Node::make_shared("grasping_cli_node");
            grasping_cli = grasping_cli_node->create_client<std_srvs::srv::SetBool>("/robot_model/switch");
            //items for robot move commanding
            this->move_group.setMaxAccelerationScalingFactor(1.0);
            this->move_group.setMaxVelocityScalingFactor(1.0);
            target_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose",rclcpp::QoS(1),std::bind(&MyRobotControl::myrobot_move,this,std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(),"Initialization successful");
            this->myrobot_make_ready();
        }
        std::shared_ptr<rclcpp::Node> grasping_cli_node;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr grasping_cli;
        moveit::planning_interface::MoveGroupInterface move_group;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub;
        geometry_msgs::msg::Pose previous_pose;
    private:
        void myrobot_make_ready()
        {
            this->move_group.setMaxVelocityScalingFactor(0.05);
            this->move_group.setMaxAccelerationScalingFactor(0.05);
            std::vector<double> ready_joints_values{0.0, 0.5, 0.4, 0.0, 0.5, 0.0};
            this->move_group.setJointValueTarget(ready_joints_values);
            if(this->move_group.move())
            {
                RCLCPP_INFO(this->get_logger(),"Ready pose achieved.");
                // geometry_msgs::msg::PoseStamped myrobot_pose = this->move_group.getCurrentPose();
                // auto myrobot_pose = std::make_shared<geometry_msgs::msg::PoseStamped>(this->move_group.getCurrentPose("link_vacuum"));
                // RCLCPP_INFO(this->get_logger(),"Current pose after made ready: %ld",myrobot_pose->pose.position.x,
                //                                                                                 myrobot_pose->pose.position.y,
                //                                                                                 myrobot_pose->pose.position.z);
            }
        }
        void myrobot_move(geometry_msgs::msg::PoseStamped::SharedPtr msg)     //const...
        {
            RCLCPP_INFO(this->get_logger(),"Move group command received.");
            this->move_group.setMaxVelocityScalingFactor(0.1);
            this->move_group.setMaxAccelerationScalingFactor(0.1);
            this->move_group.setGoalPositionTolerance(0.05);
            this->move_group.setGoalOrientationTolerance(0.05);
            this->move_group.setPlanningTime(5);
            this->move_group.setNumPlanningAttempts(10);
            this->move_group.setPlannerId("RRT");
            // this->move_group.setGoalJointTolerance(0.1);
            if(msg->pose == previous_pose)
            {
                RCLCPP_INFO(this->get_logger(),"The same pose");
                return;
            }
            RCLCPP_INFO(this->get_logger(),"Moving to new pose");
            this->move_group.setPoseTarget(msg->pose);
            this->move_group.move();
            previous_pose=msg->pose;
        }
        void gripperOn()
        {
            auto request =std::make_shared<std_srvs::srv::SetBool::Request>();
            request -> data = true;
            while (!grasping_cli->wait_for_service(1s))
            {   
                if(!rclcpp::ok())
                {
                    RCLCPP_INFO(this->get_logger(),"Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(),"service not available, waiting again...");
            }
            auto future = grasping_cli->async_send_request(request);
            if(rclcpp::spin_until_future_complete(grasping_cli_node,future) == rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "Gripper opened success: %ld", future.get()->success);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to call service to open robot gripper.");
            }
            
        }
        void gripperOff()
        {
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = false;
            while (!grasping_cli->wait_for_service(1s))
            {
                if(!rclcpp::ok())
                {
                    RCLCPP_INFO(this->get_logger(),"Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(this->get_logger(),"service not available, waiting again...");
            }
            auto future = grasping_cli->async_send_request(request);
            if(rclcpp::spin_until_future_complete(grasping_cli_node,future) == rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(),"Gripper closed success: %ld", future.get()->success);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(),"Failed to call service to close robot gripper.");
            }
        }
};


int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto myrobot_controller = std::make_shared<MyRobotControl>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(myrobot_controller);
    // std::thread([&executor](){executor.spin();}).detach();
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

