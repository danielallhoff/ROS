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
from tensorflow.keras.models import load_model
PATH = os.path.abspath('src')
print('Loading path = {}'.format(PATH))

IMG_HEIGHT = 224
IMG_WIDTH = 224

class RobotDetectionHandler:
    def __init__(self):
        self.bridge = CvBridge()
        #Send cv images back
        #self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=5)
        #Subscribe to camera of robot and receive data
        self.image_sub = rospy.Subscriber('/robot1/camera/rgb/image_raw', Image, self.callback, queue_size = 1)
        self.model = load_model(PATH+"/mobilenetssd.h5")
        #pubDest = 'cmd_vel'
        self.model.summary()
        print("Init")

    def callback(self, data):
        #Obtain images
        print("Callback call")
        try:
            #For receive key            
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")            
            resized = cv.resize(cv_image, dsize=(IMG_HEIGHT, IMG_WIDTH), interpolation=cv.INTER_CUBIC)
            image = np.asarray(resized).astype('float32')/255.0
            (rows, cols, channels) = cv_image.shape
            array = np.array(image)
            array = array.reshape((1,) + array.shape)
            #Image size to big downsize
            #Res 0-xmin 1-ymin 2-xmax 3-ymax
            res = self.model.predict(array)
            
            cv_image = cv.rectangle(resized, (res[0][0], res[0][1]), (res[0][2], res[0][3]), color=(0,255,0),thickness = 2)
            print(res)
            cv.imwrite(PATH + "/detection.jpg", cv_image)
        except Exception as e:
            print(e)
        
def main():
    print("Start")

    handler = RobotDetectionHandler()
    
    rospy.init_node('RobotDetectionHandler', anonymous=True)
    rate = rospy.Rate(20)
    try:
        print("Spin")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv.destroyAllWindows()

if __name__ == '__main__':
    main()