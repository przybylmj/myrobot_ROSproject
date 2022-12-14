import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from myrobot_interfaces.srv import GetObjectPoint
from cv_bridge import CvBridge
import cv2 as cv
from matplotlib import pyplot as plt
import numpy as np
import math
import torch
import torchvision
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation

font                   = cv.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10,30)
fontScale              = 1
fontColor              = (255,255,255)
thickness              = 1
lineType               = 2
categ={1:'oats',2:'tea',3:'crisps',4:'soup',5:'scissors',6:'pan'}

class MyRobotVision(Node):
    def __init__(self):
        super().__init__("myrobot_vision")
        self.imgLsub = self.create_subscription(Image,'robot_model/stereocamera/left/image_raw',self.imgLcallback,10)
        self.imgLsub
        self.imgRsub = self.create_subscription(Image,'robot_model/stereocamera/right/image_raw',self.imgRcallback,10)
        self.imgRsub
        self.br = CvBridge()
        self.model = torch.load('/home/jan/ws_myrobot/src/myrobot_vision/dnn/model.pth')
        self.create_service(GetObjectPoint,"point_compute",self.pointCompute)
        self.tf_buffor = Buffer()
        self.tf_listener = TransformListener(self.tf_buffor,self)
        self.world_frame = "world"
        self.camera_frame = "link_camera"
        self.B = 0.07 # Baseline
        self.f = 476 # focal (in pixels)
        self.c = 400 # geometrical image center (the same for x and y axes)
        self.thres = 0.5

    def imgLcallback(self,msg):
        self.imgL = self.br.imgmsg_to_cv2(msg,desired_encoding = 'bgra8')
        
    def imgRcallback(self,msg):
        self.imgR = self.br.imgmsg_to_cv2(msg,desired_encoding = 'bgra8')

    def cameraFrame2robotFrame(self,Pt_cameraFrame):
        ### convert stereo vision axis to left robot camera axis
        Pt_cameraFrame[0],Pt_cameraFrame[1],Pt_cameraFrame[2] = Pt_cameraFrame[2],- Pt_cameraFrame[0],- Pt_cameraFrame[1]
        now = rclpy.time.Time()
        trans = self.tf_buffor.lookup_transform(self.world_frame,self.camera_frame,now)
        transl = trans.transform.translation
        quat = trans.transform.rotation
        rot = Rotation.from_quat([quat.x,quat.y,quat.z,quat.w])
        alfa, beta, gamma = rot.as_euler('XYZ',degrees=False)

        T = np.array([[1,0,0,transl.x],[0,1,0,transl.y],[0,0,1,transl.z],[0,0,0,1]])
        Rx = np.array([[1,0,0,0],[0,math.cos(alfa),-math.sin(alfa),0],[0,math.sin(alfa),math.cos(alfa),0],[0,0,0,1]])
        Ry = np.array([[math.cos(beta),0,math.sin(beta),0],[0,1,0,0],[-math.sin(beta),0,math.cos(beta),0],[0,0,0,1]])
        Rz = np.array([[math.cos(gamma),-math.sin(gamma),0,0],[math.sin(gamma),math.cos(gamma),0,0],[0,0,1,0],[0,0,0,1]])
        M = T @ Rx @ Ry @ Rz
        Pt_cameraFrame = np.append(Pt_cameraFrame,[1])
        Pt_robotFrame = M @ Pt_cameraFrame
        Pt_robotFrame = np.delete(Pt_robotFrame,3)
        return Pt_robotFrame
        
    def pointCompute(self,request,response):
        self.get_logger().info("Stereo method called")
        imgTensL = torchvision.transforms.functional.to_tensor(cv.cvtColor(self.imgL,cv.COLOR_BGRA2RGB))
        imgTensL = imgTensL.to("cuda")
        imgTensR = torchvision.transforms.functional.to_tensor(cv.cvtColor(self.imgR,cv.COLOR_BGRA2RGB))
        imgTensR = imgTensR.to("cuda")
        resL,resR,res = [],[],[] # p2d,cat,score
        P3D = [] # p3d,cat,score
        ### imgL
        with torch.no_grad():
            self.model.eval()
            output = self.model([imgTensL])
        for idx in range(0,len(output[0]['scores'])):
            if output[0]['scores'][idx] > self.thres:
                bbox = output[0]['boxes'][idx].to("cpu")
                cat = output[0]['labels'][idx].to("cpu")
                p0 =output[0]['keypoints'][idx][0].to("cpu")
                score = output[0]['scores'][idx].to("cpu")
                resL.append([p0,cat.item(),score.item()])
        ### imgR
        with torch.no_grad():
            self.model.eval()
            output = self.model([imgTensR])
        for idx in range(0,len(output[0]['scores'])):
            if output[0]['scores'][idx] > self.thres:
                bbox = output[0]['boxes'][idx].to("cpu")
                cat = output[0]['labels'][idx].to("cpu")
                p0 =output[0]['keypoints'][idx][0].to("cpu")
                score = output[0]['scores'][idx].to("cpu")
                resR.append([p0,cat.item(),score.item()])
               
        ### sorting in relative to detection score   
        # resL = sorted(resL,key=lambda x: x[2],reverse=True)
        # resR = sorted(resR,key=lambda x: x[2],reverse=True)

        for itemL in resL:
            for itemR in resR:
                if itemL[1] == itemR[1]:
                    ### checking if point of the same category already is in the res list
                    alreadyIs = False
                    for item in res:
                        if item[2] == itemL[1]:
                            alreadyIs = True
                    if not alreadyIs:
                        res.append([itemL[0],itemR[0],itemL[1],itemL[2]])

        for item in res:
            ptL,ptR = item[0],item[1]
            dis = ptL[0] - ptR[0]
            Z = (self.B*self.f)/dis
            X = (ptL[0] - self.c)*Z/self.f
            Y = (ptL[1] - self.c)*Z/self.f
            Pt_robotFrame = self.cameraFrame2robotFrame([X,Y,Z])
            P3D.append([Pt_robotFrame,item[2],item[3]]) ### 3dp (robot frame),cat,score

        if P3D:
            P3D = sorted(P3D,key=lambda x: x[0][2],reverse=True) ### sorting results in respect to Z coor
            for p3d in P3D:
                print(p3d)
            response.object_detected = True
            response.object_point_pose_x = round(P3D[0][0][0],3)
            response.object_point_pose_y = round(P3D[0][0][1],3)
            response.object_point_pose_z = round(P3D[0][0][2],3)
            response.object_cat = int(P3D[0][1])
        else:
            response.object_detected = False
    
        return response

def main(args=None):
    rclpy.init(args=args)
    myrobot_vision = MyRobotVision()
    rclpy.spin(myrobot_vision)
    myrobot_vision.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()