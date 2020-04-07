#!/usr/bin/env python

import rospy
import cv2 as cv
import sys
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from keras import backend as K
from keras.models import Model, load_model
PATH = os.path.abspath('src')
print('Loading path = {}'.format(PATH))

IMG_HEIGHT = 28
IMG_WIDTH = 28

class RobotHandler:
    def __init__(self):
        self.bridge = CvBridge()
        #Send cv images back
        #self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=5)
        #Subscribe to camera of robot and receive data
        self.image_sub = rospy.Subscriber('/robot1/camera/rgb/image_raw', Image, self.callback, queue_size = 1)
        self.model = load_model(PATH+"/model_trained.h5")
        #pubDest = 'cmd_vel'
        self.cmd_vel_pub = rospy.Publisher('/robot1/mobile_base/commands/velocity', Twist, queue_size=1)
        print("Init")

    def callback(self, data):
        #Obtain images
        print("Callback call")
        try:
            #For receive key            
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")            
            
            (rows, cols, channels) = cv_image.shape
            #Image size to big downsize
            if cols > IMG_WIDTH and rows > IMG_HEIGHT :
                cv.circle(cv_image, (IMG_HEIGHT,IMG_WIDTH), 10, 255)
            
            res = self.model.predict(cv_image)
            twist = Twist()
            twist.linear.x = res[0]
            twist.angular.z = res[1]
            
            self.cmd_vel_pub.publish(twist)
        except Exception as e:
            print(e)
        
def main():
    print("Start")

    handler = RobotHandler()
    
    rospy.init_node('RobotHandler', anonymous=True)
    rate = rospy.Rate(60)
    try:
        print("Spin")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv.destroyAllWindows()

if __name__ == '__main__':
    main()