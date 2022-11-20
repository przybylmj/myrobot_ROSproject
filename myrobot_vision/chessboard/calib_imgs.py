import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
from matplotlib import pyplot as plt
import numpy as np
import time
from myrobot_interfaces.srv import GetObjectPoint
from rclpy.qos import qos_profile_sensor_data

class MyRobotVision(Node):
    def __init__(self):
        super().__init__("myrobot_vision")
        self.imgLsub = self.create_subscription(Image,'robot_model/stereocamera/left/image_raw',self.imgLcallback,10) #10
        self.imgLsub
        self.imgRsub = self.create_subscription(Image,'robot_model/stereocamera/right/image_raw',self.imgRcallback,10) #10
        self.imgRsub
        self.br = CvBridge()    
        self.create_service(GetObjectPoint,"point_compute",self.calibrate)

    def imgLcallback(self,msg):
        self.imgL = self.br.imgmsg_to_cv2(msg,desired_encoding = 'bgra8')
        
    def imgRcallback(self,msg):
        self.imgR = self.br.imgmsg_to_cv2(msg,desired_encoding = 'bgra8')

    def calibrate(self,request,response):
        ############
        idx = 50
        ############
        print("### START ###")
        time.sleep(1)
        cv.imwrite(f"/home/jan/ws_myrobot/src/myrobot_vision/chessboard/imgs/imgsL/imgL_{idx}.png",self.imgL)
        cv.imwrite(f"/home/jan/ws_myrobot/src/myrobot_vision/chessboard/imgs/imgsR/imgR_{idx}.png",self.imgR)

        print("### STOP ###")
        response.object_detected = True 
        return response

        

def main(args=None):
    rclpy.init(args=args)
    myrobot_vision = MyRobotVision()   
    rclpy.spin(myrobot_vision)
    myrobot_vision.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()