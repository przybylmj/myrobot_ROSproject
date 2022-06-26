import encodings
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv

class MyRobotVision(Node):
    def __init__(self):
        super().__init__("myrobot_vision")
        self.imgLsub = self.create_subscription(Image,'robot_model/stereocamera/left/image_raw',self.imgLcallback,10)
        self.imgLsub
        self.imgRsub = self.create_subscription(Image,'robot_model/stereocamera/right/image_raw',self.imgRcallback,10)
        self.imgRsub
        self.brigde = CvBridge()
        # self.saveImgs()

    def imgLcallback(self,msg):
        imgL = self.brigde.imgmsg_to_cv2(msg,desired_encoding= 'bgra8')
        cv.imshow("CameraL",imgL)
        cv.waitKey(0)

    def imgRcallback(self,msg):
        pass
        # imgR = self.brigde.imgmsg_to_cv2(msg,desired_encoding='bgra8')
        # cv.imshow("CameraR",imgR)
        # cv.waitKey(0)
        
    
    def saveImgs(self):
        imgL = self.brigde.imgmsg_to_cv2(self.imgLmsg,desired_encoding='passthrough')
        imgR = self.brigde.imgmsg_to_cv2(self.imgRmsg,desired_encoding='passthrough')
        cv.imwrite('/home/jan/ws_myrobot/src/myrobot_vision/dnn/imgL.jpg',imgL)
        cv.imwrite('/home/jan/ws_myrobot/src/myrobot_vision/dnn/imgR.jpg',imgR)

def main(args=None):
    rclpy.init(args=args)
    myrobot_vision = MyRobotVision()
    rclpy.spin(myrobot_vision)
    myrobot_vision.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()