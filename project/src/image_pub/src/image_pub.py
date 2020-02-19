#!/usr/bin/env python
#http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rospy
import cv2 as cv
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

PATH = "/home/dfa17/Escritorio/ROS/project/src/images/"

class TurtleBotHandler:
    def __init__(self):
        self.bridge = CvBridge()
        
        #Send cv images back
        #self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=5)
        #Subscribe to camera of robot and receive data
        self.image_sub = rospy.Subscriber('/robot1/camera/rgb/image_raw', Image, self.callback)
        print("Init")
    def callback(self, data):
        #Obtain images
        print("Callback call")
        try:
            #For receive key            
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")            
            cv.imshow("view", cv_image)
            #cv.imwrite(PATH + "frame_" + str(self.frame) + "_" + msg + ".jpg", cv_image)
            
            cv.waitKey(30)
        except Exception as e:
            print(e)
        print("Image sent")
        (rows, cols, channels) = cv_image.shape
        #Image size to big downsize
        if cols > 60 and rows > 60:
            cv.circle(cv_image, (50,50), 10, 255)

        cv.imshow("Image window", cv_image)
        cv.waitKey(3)
        ''' #Resend image back
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            cv.imwrite('image_test', cv_image)
        except CvBridgeError as e:
            print(e)
        '''

def main():
    print("Start")
    #Load turtlebothandler
    handler = TurtleBotHandler()
    
    rospy.init_node('TurtleBotHandler', anonymous=True)
    rate = rospy.Rate(30)
    try:
        print("Spin")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv.destroyAllWindows()

if __name__ == '__main__':
    main()