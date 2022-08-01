#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include <myrobot_interfaces/srv/set_target_pose.hpp>
#include <std_srvs/srv/set_bool.hpp>
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
            //items for robot move commanding
            this->move_group.setMaxAccelerationScalingFactor(1.0);
            this->move_group.setMaxVelocityScalingFactor(1.0);
            this->myrobot_make_ready();
            // target_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose",rclcpp::QoS(1),std::bind(&MyRobotControl::myrobot_move,this,std::placeholders::_1));
            target_pose_serv = this->create_service<myrobot_interfaces::srv::SetTargetPose>("/target_pose",std::bind(&MyRobotControl::myrobot_move,this,std::placeholders::_1,std::placeholders::_2));
            RCLCPP_INFO(this->get_logger(),"Initialization successful");
        }
        moveit::planning_interface::MoveGroupInterface move_group;
        // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub;
        rclcpp::Service<myrobot_interfaces::srv::SetTargetPose>::SharedPtr target_pose_serv;
        geometry_msgs::msg::Pose pose;
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
                // RCLCPP_INFO(this->get_logger(),"Current pose after made ready: %ld",myrobot_pose);
            }
        }
        void myrobot_move(const std::shared_ptr<myrobot_interfaces::srv::SetTargetPose::Request> request,
            std::shared_ptr<myrobot_interfaces::srv::SetTargetPose::Response> response)     //const...
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
            pose.position.x = request->object_point_pose_x;
            pose.position.y = request->object_point_pose_y;
            pose.position.z = request->object_point_pose_z;
            pose.orientation.x = request->object_point_rot_x;
            pose.orientation.y = request->object_point_rot_y;
            pose.orientation.z = request->object_point_rot_z;
            pose.orientation.w = request->object_point_rot_w;
            if(pose == previous_pose)
            {
                RCLCPP_INFO(this->get_logger(),"The same pose");
                return;
            }
            RCLCPP_INFO(this->get_logger(),"Moving to new pose");
            this->move_group.setPoseTarget(pose);
            this->move_group.move();
            previous_pose = pose;
            response->success = true;
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

