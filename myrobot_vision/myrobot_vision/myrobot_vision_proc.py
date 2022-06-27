from cv2 import imwrite
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2 as cv
from matplotlib import pyplot as plt

class MyRobotVision(Node):
    def __init__(self):
        super().__init__("myrobot_vision")
        self.imgLsub = self.create_subscription(Image,'robot_model/stereocamera/left/image_raw',self.imgLcallback,10)
        self.imgLsub
        self.imgRsub = self.create_subscription(Image,'robot_model/stereocamera/right/image_raw',self.imgRcallback,10)
        self.imgRsub
        self.br = CvBridge()
        self.create_service(SetBool,"stereo_count",self.stereoCount)

    def imgLcallback(self,msg):
        self.imgL = self.br.imgmsg_to_cv2(msg,desired_encoding = 'bgra8')
        
    def imgRcallback(self,msg):
        self.imgR = self.br.imgmsg_to_cv2(msg,desired_encoding = 'bgra8')

    def stereoCount(self,request,response):
        self.get_logger().info("Stereo method called")
        # cv.imwrite("/home/jan/ws_myrobot/src/myrobot_vision/dnn/imgL.png",self.imgL)
        # cv,imwrite("/home/jan/ws_myrobot/src/myrobot_vision/dnn/imgR.png",self.imgR)
        # cv.imshow("CameraRGray",cv.cvtColor(self.imgR,cv.COLOR_BGRA2GRAY))
        # cv.waitKey(0)
        stereo = cv.StereoBM_create(numDisparities=16,blockSize=15)
        disparity = stereo.compute(cv.cvtColor(self.imgR,cv.COLOR_BGRA2GRAY),cv.cvtColor(self.imgL,cv.COLOR_BGRA2GRAY))
        plt.imshow(disparity,'gray')
        plt.show()
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