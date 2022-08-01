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
        self.Q = np.array([[1,0,0,0],[0,1,0,0],[0,0,0,480],[0,0,-14.2857,0]])
        self.thres = 0.1
        self.stereo = cv.StereoBM_create(numDisparities=16,blockSize=15)

    def imgLcallback(self,msg):
        self.imgL = self.br.imgmsg_to_cv2(msg,desired_encoding = 'bgra8')
        
    def imgRcallback(self,msg):
        self.imgR = self.br.imgmsg_to_cv2(msg,desired_encoding = 'bgra8')

    def cameraFrame2robotFrame(self,Pt_cameraFrame):
        now = rclpy.time.Time()
        trans = self.tf_buffor.lookup_transform(self.world_frame,self.camera_frame,now)
        transl = trans.transform.translation
        quat = trans.transform.rotation
        t0 = +2.0 * (quat.w * quat.x + quat.y * quat.z)
        t1 = +1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y)
        alfa = math.atan2(t0, t1)
        t2 = +2.0 * (quat.w * quat.y - quat.z * quat.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        beta = math.asin(t2)
        t3 = +2.0 * (quat.w * quat.z + quat.x * quat.y)
        t4 = +1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        gamma = math.atan2(t3, t4)
        T = np.array([[1,0,0,transl.x],[0,1,0,transl.y],[0,0,1,transl.z],[0,0,0,1]])
        Rx = np.array([[1,0,0,0],[0,math.cos(alfa),-math.sin(alfa),0],[0,math.sin(alfa),math.cos(alfa),0],[0,0,0,1]])
        Ry = np.array([[math.cos(beta),0,math.sin(beta),0],[0,1,0,0],[-math.sin(beta),0,math.cos(beta),0],[0,0,0,1]])
        Rz = np.array([[math.cos(gamma),-math.sin(gamma),0,0],[math.sin(gamma),math.cos(gamma),0,0],[0,0,1,0],[0,0,0,1]])
        M = T @ Rx @ Ry @ Rz
        Pt_cameraFrame = np.append(Pt_cameraFrame,[1])
        Pt_robotFrame = T @ Pt_cameraFrame
        Pt_robotFrame = np.delete(Pt_robotFrame,3)
        return Pt_robotFrame
        


    def pointCompute(self,request,response):
        self.get_logger().info("Stereo method called")
        # cv.imwrite("/home/jan/ws_myrobot/src/myrobot_vision/dnn/imgL.png",self.imgL)
        # cv.imwrite("/home/jan/ws_myrobot/src/myrobot_vision/dnn/imgR.png",self.imgR)
        # cv.imshow("CameraRGray",cv.cvtColor(self.imgL,cv.COLOR_BGRA2GRAY))
        # cv.imshow("CameraRGray",self.imgL)
        # cv.waitKey(0)
        disparity = self.stereo.compute(cv.cvtColor(self.imgR,cv.COLOR_BGRA2GRAY),cv.cvtColor(self.imgL,cv.COLOR_BGRA2GRAY))
        # plt.imshow(disparity,'gray')
        # plt.show()
        img3d = cv.reprojectImageTo3D(disparity,self.Q,handleMissingValues=True)
        # for x in img3d:
        #     for y in x:
        #         if y[2] != 1.0000000e+04:
        #             print(y)
        imgTens = torchvision.transforms.functional.to_tensor(cv.cvtColor(self.imgL,cv.COLOR_BGRA2RGB))
        with torch.no_grad():
            self.model.eval()
            output = self.model([imgTens])
        if output[0]['scores'][0] > self.thres:
            response.object_detected = True 
            bbox = output[0]['boxes'][0]
            cat = output[0]['labels'][0]
            p0 =output[0]['keypoints'][0][0]
            p1 =output[0]['keypoints'][0][1]
            p2 =output[0]['keypoints'][0][2]
            p3 =output[0]['keypoints'][0][3]
            p4 =output[0]['keypoints'][0][4]
            graspPt_cameraFrame = img3d[int(p0[0])][int(p0[1])]
            # self.imgL= cv.rectangle(self.imgL,(int(bbox[0].item()),int(bbox[1].item())),(int(bbox[2].item()),int(bbox[3].item())),(255,0,255),3)
            # self.imgL= cv.circle(self.imgL,(int(p0[0]),int(p0[1])),4,(255,255,255),5) #grasping point
            # self.imgL= cv.circle(self.imgL,(int(p1[0]),int(p1[1])),4,(255,255,0),5)
            # self.imgL= cv.circle(self.imgL,(int(p2[0]),int(p2[1])),4,(255,255,0),5)
            # self.imgL= cv.circle(self.imgL,(int(p3[0]),int(p3[1])),4,(255,255,0),5)
            # self.imgL= cv.circle(self.imgL,(int(p4[0]),int(p4[1])),4,(255,255,0),5)
            # cv.imshow("DnnTest",self.imgL)
            # cv.waitKey(0)
            graspPt_robotFrame = self.cameraFrame2robotFrame(graspPt_cameraFrame)
            response.object_point_pose_x = round(graspPt_robotFrame[0],3)
            response.object_point_pose_y = round(graspPt_robotFrame[1],3)
            response.object_point_pose_z = round(graspPt_robotFrame[2],3)
            response.object_cat = int(cat)
            print(graspPt_robotFrame)
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