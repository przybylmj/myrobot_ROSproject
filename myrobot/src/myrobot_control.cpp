#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

const std::string MOVE_GROUP = "myrobot_arm";

class MyRobotControl : public rclcpp::Node
{
    public:
        MyRobotControl(): Node("myrobot_control"), move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)),MOVE_GROUP)
        {
            this->move_group_.setMaxAccelerationScalingFactor(1.0);
            this->move_group_.setMaxVelocityScalingFactor(1.0);
            target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose",rclcpp::QoS(1),std::bind(&MyRobotControl::myrobot_move,this,std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(),"Initialization successful");
        } 
        moveit::planning_interface::MoveGroupInterface move_group_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
        geometry_msgs::msg::Pose previous_pose;
    private:
        void myrobot_move(geometry_msgs::msg::PoseStamped::SharedPtr msg)     //const...
        {
            if(msg->pose == previous_pose)
            {
                RCLCPP_INFO(this->get_logger(),"The same pose");
                return;
            }
            RCLCPP_INFO(this->get_logger(),"Moving to new pose");
            this->move_group_.setPoseTarget(msg->pose);
            this->move_group_.move();
            previous_pose=msg->pose;
        }
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc,argv);
    auto myrobot_controller = std::make_shared<MyRobotControl>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(myrobot_controller);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}