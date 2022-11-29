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
        ### transl
        pose.object_point_pose_x = 0.7
        pose.object_point_pose_y = 0.05 #-0.07
        pose.object_point_pose_z = 0.7
        ### rotate
        pose.object_point_rot_x = -0.044
        pose.object_point_rot_y = 0.498
        pose.object_point_rot_z = -0.075
        pose.object_point_rot_w = 0.863
        # ### transl
        # pose.object_point_pose_x = 1.2
        # pose.object_point_pose_y = 0.0 #-0.07
        # pose.object_point_pose_z = 0.7
        # ### rotate
        # pose.object_point_rot_x = 0.0
        # pose.object_point_rot_y = 0.500
        # pose.object_point_rot_z = 0.0
        # pose.object_point_rot_w = 0.866
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
        #### ???????????  #####
        print(f"x:  {blackboard.obj_point_info.object_point_pose_x}")
        print(f"y:  {blackboard.obj_point_info.object_point_pose_y}")
        print(f"z:  {blackboard.obj_point_info.object_point_pose_z}")
        pose.object_point_pose_x = blackboard.obj_point_info.object_point_pose_x
        pose.object_point_pose_y = blackboard.obj_point_info.object_point_pose_y
        pose.object_point_pose_z = blackboard.obj_point_info.object_point_pose_z + 0.1 # safety offset
        ### approaching gripper pose 87 degrees
        pose.object_point_rot_x = 0.0
        pose.object_point_rot_y = 0.682
        pose.object_point_rot_z = 0.0
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
        item.data = False #!!!
        return item

    def grasp_object_res(self,blackboard,response):
        return "grasped"

class AuxPose(ServiceState):
    def __init__(self):
        super().__init__(Node("AuxPoseState"),SetTargetPose,'/target_pose',self.aux_pose_req,["pose_set"],self.aux_pose_res)

    def aux_pose_req(self,blackboard):
        print("AuxPose State Processing")
        pose = SetTargetPose.Request()
        pose.object_point_pose_x = 1.0
        pose.object_point_pose_y = 0.0 #-0.07
        pose.object_point_pose_z = 0.95
        ### rotate
        pose.object_point_rot_x = 0.0
        pose.object_point_rot_y = 0.682
        pose.object_point_rot_z = 0.0
        pose.object_point_rot_w = 0.732
        return pose

    def aux_pose_res(self,blackboard,response):
        return "pose_set"

class DesPose(ServiceState):
    def __init__(self):
        super().__init__(Node("DesPoseState"),SetTargetPose,"/target_pose",self.des_pose_req,["pose_set"],self.des_pose_res)

    def des_pose_req(self,blackboard):
        print("DesPose State Processing")
        pose = SetTargetPose.Request()
        if blackboard.obj_point_info.object_cat == 1: ### oats
            pose.object_point_pose_x = 0.5
            pose.object_point_pose_y = 0.7
            pose.object_point_pose_z = 0.55
        if blackboard.obj_point_info.object_cat == 2: ### tea
            pose.object_point_pose_x = 0.5
            pose.object_point_pose_y = -0.7
            pose.object_point_pose_z = 0.55
        if blackboard.obj_point_info.object_cat == 3: ### cookies
            pose.object_point_pose_x = 0.2
            pose.object_point_pose_y = -0.8
            pose.object_point_pose_z = 0.55
        if blackboard.obj_point_info.object_cat == 4: # soup
            pose.object_point_pose_x = 0.2
            pose.object_point_pose_y = -0.8
            pose.object_point_pose_z = 0.55
        if blackboard.obj_point_info.object_cat == 5: # scissors
            pose.object_point_pose_x = -0.25
            pose.object_point_pose_y = -0.8
            pose.object_point_pose_z = 0.55
        if blackboard.obj_point_info.object_cat == 6: # pan
            pose.object_point_pose_x = -0.25
            pose.object_point_pose_y = -0.8
            pose.object_point_pose_z = 0.55
        pose.object_point_rot_x = 0.0
        pose.object_point_rot_y = 0.682
        pose.object_point_rot_z = 0.0
        pose.object_point_rot_w = 0.732
        return pose

    def des_pose_res(self,blackboard,response):
        return "pose_set"

class UngraspObject(ServiceState):
    def __init__(self):
        super().__init__(Node("UngraspObjectState"),SetBool,"robot_model1/switch",self.ungrasp_obj_req,["ungrasped"],self.ungrasp_obj_res)

    def ungrasp_obj_req(self,blackboard):
        print("UngraspObject State Processing")
        item = SetBool.Request()
        item.data = False
        return item

    def ungrasp_obj_res(self,blackboard,response):
        return "ungrasped"

class MyRobotStateMachine(Node):
    def __init__(self):
        super().__init__("state_machine_node")
        sm = StateMachine(outcomes=["finish"])
        sm.add_state("STEREO_POSE",StereoPose(),transitions={"stereo_pose_set":"STEREO_COMPUTE"})
        sm.add_state("STEREO_COMPUTE",StereoCompute(),transitions={"point_got":"GRASP_POSE","no_objects":"finish"})
        # sm.add_state("GRASP_POSE",GraspPose(),transitions={"pose_set":"GRASP_OBJ"})
        sm.add_state("GRASP_POSE",GraspPose(),transitions={"pose_set":"AUX_POSE"})
        sm.add_state("GRASP_OBJ",GraspObject(),transitions={"grasped":"AUX_POSE"})
        sm.add_state("AUX_POSE",AuxPose(),transitions={"pose_set":"DES_POSE"})
        # sm.add_state("DES_POSE",DesPose(),transitions={"pose_set":"UNGRASP_OBJ"})
        sm.add_state("DES_POSE",DesPose(),transitions={"pose_set":"STEREO_POSE"})
        sm.add_state("UNGRASP_OBJ",UngraspObject(),transitions={"ungrasped":"STEREO_POSE"})
        outcome = sm()
        print(outcome)

def main(args = None):
    rclpy.init(args=args)
    myrobot_sm = MyRobotStateMachine()
    myrobot_sm.join_spin()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


        