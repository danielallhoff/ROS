#!/usr/bin/env python
import rospy
import cv2 as cv
import sys
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import message_filters

PATH = os.path.abspath('images')

PATH = PATH + "/object_detection"
print('Saving path = {}'.format(PATH))

class Recorder:
    def __init__(self):
        self.bridge = CvBridge()
        self.dir_image_sub = rospy.Subscriber('/robot1/camera/rgb/image_raw', Image, callback=self.save_image)
        self.frame = 0

    def save_image(self, data):
        ''' Saves each image recieved'''
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")   
        cv.imwrite(PATH + "_frame_" + str(self.frame) + ".jpg", cv_image)   
        self.frame += 1

def main():
    handler = Recorder()
    rospy.init_node('Recorder', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
