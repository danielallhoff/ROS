#!/usr/bin/env python
import rospy
import cv2 as cv
import sys
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

PATH = os.path.abspath('images')
PATH = PATH + "/images"
print('Saving path = {}'.format(PATH))

class Gatherer:
    def __init__(self):
        self.frame = 0
        self.bridge = CvBridge()
        self.cmd_sub = rospy.Subscriber('cmd_key', String, self.save_key, queue_size = 1)
        self.image_sub = rospy.Subscriber('image', Image, self.save_image, queue_size = 1)

    def save_key(self, command):
        ''' Saves command key into file'''
        cadena = str(command)
        with open('command_keys.txt', 'a') as out:
            splitted = cadena.split("\"")
            out.write(splitted[1] + str('\n'))

    def save_image(self, data):
        ''' Saves each image recieved'''
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")   
        cv.imwrite(PATH + "_frame_" + str(self.frame) + ".jpg", cv_image)   
        self.frame += 1

def main():
    handler = Gatherer()
    rospy.init_node('Gatherer', anonymous=True)
    rate = rospy.Rate(2)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
