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

    ### IMAGE CAPTURE POSE #############
    def sendPose(self):
        self.req.object_point_pose_x = 1.2 #1.0
        self.req.object_point_pose_y = 0.0 #0.07
        self.req.object_point_pose_z = 0.7 #0.95

        self.req.object_point_rot_x = 0.0
        self.req.object_point_rot_y = 0.500
        self.req.object_point_rot_z = 0.0
        self.req.object_point_rot_w = 0.866
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self,self.future)
        return self.future.result()

    ### IMAGE CAPTURE POSE 01 #############
    # def sendPose(self):
    #     self.req.object_point_pose_x = 0.7
    #     self.req.object_point_pose_y = 0.05
    #     self.req.object_point_pose_z = 0.7

    #     self.req.object_point_rot_x = -0.044
    #     self.req.object_point_rot_y = 0.498
    #     self.req.object_point_rot_z = -0.075
    #     self.req.object_point_rot_w = 0.863
    #     self.future = self.cli.call_async(self.req)
    #     rclpy.spin_until_future_complete(self,self.future)
    #     return self.future.result()


    ### BASIC POSE ###################

    # def sendPose(self):
    #     self.req.object_point_pose_x = 0.8
    #     self.req.object_point_pose_y = -0.15
    #     self.req.object_point_pose_z = 0.6
    #     # self.req.object_point_rot_x = 0.0
    #     self.req.object_point_rot_y = 0.682
    #     # self.req.object_point_rot_z = 0.0
    #     self.req.object_point_rot_w = 0.732
    #     self.future = self.cli.call_async(self.req)
    #     rclpy.spin_until_future_complete(self,self.future)
    #     return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    moveit_test = MoveitTestClass()
    response = moveit_test.sendPose()
    moveit_test.get_logger().info(f"Success: {response}")
    moveit_test.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()