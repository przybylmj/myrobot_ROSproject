import rclpy
from rclpy.node import Node
from myrobot_interfaces.srv import SetTargetPose

class InterfaceTest(Node):
    def __init__(self):
        super().__init__("interface_test_node")
        self.srv = self.create_service(SetTargetPose,"/target_pose",self.server)

    def server(self,request,response):
        self.get_logger().info(f"Request info from client: {request}")
        response.success = True
        return response

def main(args = None):
    rclpy.init(args=args)
    interface_test = InterfaceTest()
    rclpy.spin(interface_test)
    interface_test.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
