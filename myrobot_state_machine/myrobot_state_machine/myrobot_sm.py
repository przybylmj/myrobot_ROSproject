from curses import flash
import rclpy
from simple_node import Node
from yasmin import State
from yasmin import StateMachine
from yasmin_ros import ServiceState
from yasmin.blackboard import Blackboard
from myrobot_interfaces.srv import SetTargetPose
from myrobot_interfaces.srv import GetObjectPoint
from std_srvs.srv import SetBool

class StereoPose(ServiceState):
    def __init__(self):
        super().__init__(Node("StereoPoseState"), SetTargetPose, '/target_pose', self.stereo_pose_req, ["stereo_pose_set"], self.stereo_pose_res)
    
    def stereo_pose_req(self,blackboard):
        print("StereoPose State Processing")
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
        return "stereo_pose_set"

class StereoCompute(ServiceState):
    def __init__(self):
        super().__init__(Node('StereoComputeState'), GetObjectPoint, '/point_compute', self.stereo_compute_req, ["point_got","no_objects"], self.stereo_compute_res)
    
    def stereo_compute_req(self,blackboard):
        print("StereoCompute State Processing")
        item = GetObjectPoint.Request()
        item.trigger = True
        return item

    def stereo_compute_res(self,blackboard,response):
        print(response)
        blackboard.obj_point_info = response
        if response.object_detected:
            return "point_got"
        else:
            print("No Object Detected!")
            return "no_objects"

class GraspPose(ServiceState):
    def __init__(self):
        super().__init__(Node("GraspPoseState"),SetTargetPose,'/target_pose',self.grasp_pose_req,["pose_set"],self.grasp_pose_res)

    def grasp_pose_req(self,blackboard):
        print("GraspPose State Processing")
        # test grasping pose
        pose = SetTargetPose.Request()
        pose.object_point_pose_x = 0.9
        pose.object_point_pose_y = -0.07
        pose.object_point_pose_z = 0.47
        # pose.object_point_rot_x = 0.0
        pose.object_point_rot_y = 0.682
        # pose.object_point_rot_z = 0.0
        pose.object_point_rot_w = 0.732
        return pose

    def grasp_pose_res(self,blackboard,response):
        return "pose_set"

class GraspObject(ServiceState):
    def __init__(self):
        super().__init__(Node("GraspObjectState"),SetBool,"/robot_model1/switch",self.grasp_object_req,["grasped"],self.grasp_object_res)

    def grasp_object_req(self,blackboard):
        print("GraspObject State Processing")
        item = SetBool.Request()
        item.data = True
        return item

    def grasp_object_res(self,blackboard,response):
        return "grasped"

class AuxPose(ServiceState):
    def __init__(self):
        super().__init__(Node("AuxPoseState"),SetTargetPose,'/target_pose',self.aux_pose_req,["pose_set"],self.aux_pose_res)

    def aux_pose_req(self,blackboard):
        print("AuxPose State Processing")
        pose = SetTargetPose.Request()
        pose.object_point_pose_x = 0.9
        pose.object_point_pose_y = -0.07
        pose.object_point_pose_z = 0.7
        # pose.object_point_rot_x = 0.0
        pose.object_point_rot_y = 0.682
        # pose.object_point_rot_z = 0.0
        pose.object_point_rot_w = 0.732
        return pose

    def aux_pose_res(self,blackboard,response):
        return "pose_set"

class MyRobotStateMachine(Node):
    def __init__(self):
        super().__init__("state_machine_node")
        sm = StateMachine(outcomes=["finish"])
        sm.add_state("STEREO_POSE  ",StereoPose(),transitions={"stereo_pose_set":"STEREO_COMPUTE"})
        sm.add_state("STEREO_COMPUTE",StereoCompute(),transitions={"point_got":"GRASP_POSE","no_objects":"finish"})
        sm.add_state("GRASP_POSE",GraspPose(),transitions={"pose_set":"GRASP_OBJ"})
        sm.add_state("GRASP_OBJ",GraspObject(),transitions={"grasped":"AUX_POSE"})
        sm.add_state("AUX_POSE",AuxPose(),transitions={"pose_set":"finish"})
        outcome = sm()
        print(outcome)

def main(args = None):
    rclpy.init(args=args)
    myrobot_sm = MyRobotStateMachine()
    myrobot_sm.join_spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


        