#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

class MyRobotControl : public rclcpp::Node
{
    public:
        MyRobotControl() : Node("myrobot_control")
        {
            grasping_cli_node  = rclcpp::Node::make_shared("grasping_cli_node");
            grasping_cli = grasping_cli_node->create_client<std_srvs::srv::SetBool>("/robot_model/switch");
            this->gripperOff();
        }
        std::shared_ptr<rclcpp::Node> grasping_cli_node;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr grasping_cli;
    private:
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
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

