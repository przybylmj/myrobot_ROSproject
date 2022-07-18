import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2 as cv
from matplotlib import pyplot as plt
import numpy as np
import torch
import torchvision
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from tf2_ros.buffer_interface import Duration, Time
from tf2_ros.buffer_interface import BufferInterface



class MyRobotVision(Node):
    def __init__(self):
        super().__init__("myrobot_vision")
        self.imgLsub = self.create_subscription(Image,'robot_model/stereocamera/left/image_raw',self.imgLcallback,10)
        self.imgLsub
        self.imgRsub = self.create_subscription(Image,'robot_model/stereocamera/right/image_raw',self.imgRcallback,10)
        self.imgRsub
        self.br = CvBridge()
        self.model = torch.load('/home/jan/ws_myrobot/src/myrobot_vision/dnn/model.pth')
        self.create_service(SetBool,"point_compute",self.pointCompute)
        self.tf_buffor = Buffer()
        self.tf_buffInter = BufferInterface()
        self.tf_listener = TransformListener(self.tf_buffor,self)
        self.tf_buffInterList = TransformListener(self.tf_buffInter,self)
        self.world_frame = "world"
        self.camera_frame = "link_vacuum"

    def imgLcallback(self,msg):
        self.imgL = self.br.imgmsg_to_cv2(msg,desired_encoding = 'bgra8')
        
    def imgRcallback(self,msg):
        self.imgR = self.br.imgmsg_to_cv2(msg,desired_encoding = 'bgra8')

    def pointCompute(self,request,response):
        Q = np.array([[1,0,0,0],[0,1,0,0],[0,0,0,480],[0,0,-14.2857,0]])
        self.get_logger().info("Stereo method called")
        # cv.imwrite("/home/jan/ws_myrobot/src/myrobot_vision/dnn/imgL.png",self.imgL)
        # cv.imwrite("/home/jan/ws_myrobot/src/myrobot_vision/dnn/imgR.png",self.imgR)
        # cv.imshow("CameraRGray",cv.cvtColor(self.imgL,cv.COLOR_BGRA2GRAY))
        # cv.imshow("CameraRGray",self.imgL)
        # cv.waitKey(0)
        stereo = cv.StereoBM_create(numDisparities=16,blockSize=15)
        disparity = stereo.compute(cv.cvtColor(self.imgR,cv.COLOR_BGRA2GRAY),cv.cvtColor(self.imgL,cv.COLOR_BGRA2GRAY))
        # plt.imshow(disparity,'gray')
        # plt.show()
        img3d = cv.reprojectImageTo3D(disparity,Q,handleMissingValues=True)
        # for x in img3d:
        #     for y in x:
        #         if y[2] != 1.0000000e+04:
        #             print(y)
        imgTens = torchvision.transforms.functional.to_tensor(cv.cvtColor(self.imgL,cv.COLOR_BGRA2RGB))
        with torch.no_grad():
            self.model.eval()
            output = self.model([imgTens])
        bbox = output[0]['boxes'][0]
        # cat = output[0]['labels'][0]
        p0 =output[0]['keypoints'][0][0]
        p1 =output[0]['keypoints'][0][1]
        p2 =output[0]['keypoints'][0][2]
        p3 =output[0]['keypoints'][0][3]
        p4 =output[0]['keypoints'][0][4]
        graspPt_cameraCoor = img3d[int(p0[0])][int(p0[1])]
        print(graspPt_cameraCoor[0])
        print(graspPt_cameraCoor[1])
        print(graspPt_cameraCoor[2])
        # self.imgL= cv.rectangle(self.imgL,(int(bbox[0].item()),int(bbox[1].item())),(int(bbox[2].item()),int(bbox[3].item())),(255,0,255),3)
        # self.imgL= cv.circle(self.imgL,(int(p0[0]),int(p0[1])),4,(255,255,255),5) #grasping point
        # self.imgL= cv.circle(self.imgL,(int(p1[0]),int(p1[1])),4,(255,255,0),5)
        # self.imgL= cv.circle(self.imgL,(int(p2[0]),int(p2[1])),4,(255,255,0),5)
        # self.imgL= cv.circle(self.imgL,(int(p3[0]),int(p3[1])),4,(255,255,0),5)
        # self.imgL= cv.circle(self.imgL,(int(p4[0]),int(p4[1])),4,(255,255,0),5)
        # cv.imshow("DnnTest",self.imgL)
        # cv.waitKey(0)
        graspPt_cameraFrame = PoseStamped()
        graspPt_cameraFrame.header.frame_id = self.camera_frame
        graspPt_cameraFrame.header.stamp = self.get_clock().now().to_msg()
        graspPt_cameraFrame.pose.position.x = graspPt_cameraCoor[0]
        graspPt_cameraFrame.pose.position.y = graspPt_cameraCoor[1]
        graspPt_cameraFrame.pose.position.z = graspPt_cameraCoor[2]

       

        
        response.success = True
        return response


        

def main(args=None):
    rclpy.init(args=args)
    myrobot_vision = MyRobotVision()
    rclpy.spin(myrobot_vision)
    myrobot_vision.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()