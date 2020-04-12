#!/usr/bin/env python

import rospy
import cv2 as cv
import sys
import os
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from keras import backend as K
#from keras.models import Model
from tensorflow.keras.models import load_model
from keras.preprocessing.sequence import TimeseriesGenerator
PATH = os.path.abspath('src')
print('Loading path = {}'.format(PATH))

IMG_HEIGHT = 48
IMG_WIDTH = 48
SEQ_LENGTH = 1

class RobotHandler:
    def __init__(self):
        self.bridge = CvBridge()
        #Send cv images back
        #self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=5)
        #Subscribe to camera of robot and receive data
        self.image_sub = rospy.Subscriber('/robot1/camera/rgb/image_raw', Image, self.callback, queue_size = 1)
        self.model = load_model(PATH+"/resnet_trained.h5")
        #pubDest = 'cmd_vel'
        self.cmd_vel_pub = rospy.Publisher('/robot1/mobile_base/commands/velocity', Twist, queue_size=1)
        print("Init")
        self.current_data = []

    def callback(self, data):
        try:
            #For receive key            
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")            
            
            (rows, cols, channels) = cv_image.shape
            #Image size to big downsize
            resized = cv.resize(cv_image, dsize=(IMG_HEIGHT, IMG_WIDTH), interpolation=cv.INTER_CUBIC)
            image = np.asarray(resized).astype('float32')/255.0
            
            self.current_data.append(image)
            if len(self.current_data) < SEQ_LENGTH:
                return

            array = np.array(self.current_data)
            #array = array.reshape((1,) + array.shape)
            res = self.model.predict(array)
            print(res)
            twist = Twist()
            twist.linear.x = res[0][0]
            twist.angular.z = res[0][1]
            
            self.cmd_vel_pub.publish(twist)
            self.current_data.pop(0)
        except Exception as e:
            print(e)
            self.current_data.pop(0)
        
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