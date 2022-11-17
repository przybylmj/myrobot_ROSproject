from turtle import shape
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
        self.Q = np.array([[1,0,0,0],[0,1,0,0],[0,0,0,480],[0,0,-14.2857,0]])
        self.thres = 0.1
        # self.stereo = cv.StereoBM_create(numDisparities=16,blockSize=15)
        # self.stereo = cv.StereoBM_create(numDisparities=16,blockSize=15)
        self.stereo = cv. StereoSGBM_create(numDisparities=16,blockSize=3)

    def imgLcallback(self,msg):
        self.imgL = self.br.imgmsg_to_cv2(msg,desired_encoding = 'bgra8')
        
    def imgRcallback(self,msg):
        self.imgR = self.br.imgmsg_to_cv2(msg,desired_encoding = 'bgra8')

    def cameraFrame2robotFrame(self,Pt_cameraFrame):
        now = rclpy.time.Time()
        trans = self.tf_buffor.lookup_transform(self.world_frame,self.camera_frame,now)
        transl = trans.transform.translation
        quat = trans.transform.rotation
        rot = Rotation.from_quat([quat.x,quat.y,quat.z,quat.w])
        alfa, beta, gamma = rot.as_euler('XYZ',degrees=False)
        ### write camera pose to file
        # with open("vision_test/camera_pose.txt",'w') as camera_pose_file:
        #     camera_pose_file.write(f"transl (x,y,z):    {transl.x}  {transl.y}  {transl.z}")
        #     camera_pose_file.write(f"rot (quat: x,y,z,w):   {quat.x}    {quat.y}    {quat.z}    {quat.w}")
        #     camera_pose_file.close()
        print(f"transl (x,y,z):    {transl.x}  {transl.y}  {transl.z}")
        print(f"rot (quat: x,y,z,w):   {quat.x}    {quat.y}    {quat.z}    {quat.w}")

        # t0 = +2.0 * (quat.w * quat.x + quat.y * quat.z)
        # t1 = +1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y)
        # alfa = math.atan2(t0, t1)
        # t2 = +2.0 * (quat.w * quat.y - quat.z * quat.x)
        # t2 = +1.0 if t2 > +1.0 else t2
        # t2 = -1.0 if t2 < -1.0 else t2
        # beta = math.asin(t2)
        # t3 = +2.0 * (quat.w * quat.z + quat.x * quat.y)
        # t4 = +1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        # gamma = math.atan2(t3, t4)
        print("Quat: ",[quat.x,quat.y,quat.z,quat.w])
        print(f"Alfa, beta, gamma:  {alfa}  {beta}  {gamma}")
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
        cv.imwrite("/home/jan/ws_myrobot/src/myrobot_vision/myrobot_vision/vision_test/imgL.png",self.imgL)
        cv.imwrite("/home/jan/ws_myrobot/src/myrobot_vision/myrobot_vision/vision_test/imgR.png",self.imgR)
        # cv.imshow("CameraRGray",cv.cvtColor(self.imgL,cv.COLOR_BGRA2GRAY))
        # cv.imshow("CameraRGray",self.imgL)
        # cv.waitKey(0)
        disparity = self.stereo.compute(cv.cvtColor(self.imgR,cv.COLOR_BGRA2GRAY),cv.cvtColor(self.imgL,cv.COLOR_BGRA2GRAY))
        # plt.imshow(disparity,'gray')
        # plt.show()
        img3d = cv.reprojectImageTo3D(disparity,self.Q,handleMissingValues=True)
        img3d_array = np.array(img3d)
        np.save("point_cloud_matrix.npy",img3d_array)
        ### TO DO ### get depth map from points cloud 
        imgTens = torchvision.transforms.functional.to_tensor(cv.cvtColor(self.imgL,cv.COLOR_BGRA2RGB))
        imgTens = imgTens.to("cuda")
        with torch.no_grad():
            self.model.eval()
            output = self.model([imgTens])
        if output[0]['scores'][0] > self.thres:
            for idx in range(0,1):
                response.object_detected = True 
                bbox = output[0]['boxes'][idx]
                cat = output[0]['labels'][idx]
                p0 =output[0]['keypoints'][idx][0]
                print("grasping point CameraFrame:  ",p0)
            
                graspPt_cameraFrame = img3d[int(p0[0])][int(p0[1])]
                print("3D point value: ",graspPt_cameraFrame)
                self.imgL= cv.rectangle(self.imgL,(int(bbox[0].item()),int(bbox[1].item())),(int(bbox[2].item()),int(bbox[3].item())),(255,0,255),3)
                self.imgL= cv.circle(self.imgL,(int(p0[0]),int(p0[1])),4,(255,255,255),5) #grasping point
                cv.putText(self.imgL,categ[int(cat.item())],(int(bbox[0].item())+40,int(bbox[1].item())) , font, fontScale,fontColor,thickness,lineType)

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