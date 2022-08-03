import rclpy
from simple_node import Node
from yasmin import State
from yasmin import StateMachine
from yasmin_ros import ServiceState
from yasmin.blackboard import Blackboard
from myrobot_interfaces.srv import SetTargetPose
from myrobot_interfaces.srv import GetObjectPoint

class StereoPose(ServiceState):
    def __init__(self):
        super().__init__(Node("StereoPoseSM"), SetTargetPose, '/target_pose', self.stereo_pose_req, ["pose_set"], self.stereo_pose_res)
    
    def stereo_pose_req(self,blackboard):
        pose = SetTargetPose.Request()
        pose.object_point_pose_x = 0.9
        pose.object_point_pose_y = -0.07
        pose.object_point_pose_z = 0.7
        # pose.object_point_rot_x = 0.0
        pose.object_point_rot_y = 0.682
        # pose.object_point_rot_z = 0.0
        pose.object_point_rot_w = 0.732
        return pose

    def stereo_pose_res(self,blackboard,response):
        blackboard.stereo_pose_success = response.success
        return "pose_set"

class StereoCompute(ServiceState):
    def __init__(self):
        super().__init__(Node('StereoComputeSM'), GetObjectPoint, '/point_compute', self.stereo_compute_req, ["point_got"], self.stereo_compute_res)

    def stereo_compute_req(self,blackboard):
        print(f"Info from stereo pose state: {blackboard.stereo_pose_success}")
        trigger = GetObjectPoint.Request()
        trigger = True
        return trigger

    def stereo_compute_res(self,blackboard,response):
        blackboard.obj_point_info = response
        return "point_got"

class MyRobotStateMachine(Node):
    def __init__(self):
        super().__init__("state_machine_node")
        sm = StateMachine(outcomes=["finish"])
        sm.add_state("StereoPose",StereoPose(),transitions={"pose_set":"StereoCompute"})
        sm.add_state("StereoCompute",StereoCompute(),transitions={"point_got":"finish"})

        outcome = sm()
        print(outcome)

def main(args = None):
    rclpy.init(args=args)
    myrobot_sm = MyRobotStateMachine()
    myrobot_sm.join_spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


        