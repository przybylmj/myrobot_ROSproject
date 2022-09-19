import rclpy
from rclpy.node import Node
from myrobot_interfaces.srv import SetTargetPose

class MoveitTestClass(Node):
    def __init__(self):
        super().__init__("moveit_test_cli")
        self.cli = self.create_client(SetTargetPose,"/target_pose")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetTargetPose.Request()

    # def sendPose(self):
    #     self.req.target_pose.pose.position.x = 0.9
    #     self.req.target_pose.pose.position.y = -0.07
    #     self.req.target_pose.pose.position.z = 0.7
    #     # self.req.target_pose.pose.orientation.x = 0.0
    #     self.req.target_pose.pose.orientation.y = 0.682
    #     # self.req.target_pose.pose.orientation.z = 0.0
    #     self.req.target_pose.pose.orientation.w = 0.732
    #     self.future = self.cli.call_async(self.req)
    #     rclpy.spin_until_future_complete(self,self.future)
    #     return self.future.result()

    def sendPose(self):
        self.req.object_point_pose_x = 0.8
        self.req.object_point_pose_y = -0.15
        self.req.object_point_pose_z = 0.8
        # self.req.object_point_rot_x = 0.0
        self.req.object_point_rot_y = 0.682
        # self.req.object_point_rot_z = 0.0
        self.req.object_point_rot_w = 0.732
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    moveit_test = MoveitTestClass()
    response = moveit_test.sendPose()
    moveit_test.get_logger().info(f"Success: {response}")
    moveit_test.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()