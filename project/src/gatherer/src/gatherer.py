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
        #self.image_sub = rospy.Subscriber('/robot1/camera/rgb/image_raw', Image, self.Gather, queue_size = 1, buff_size = 2**30)
        self.frame = 0
        self.bridge = CvBridge()
        self.cmd_sub = rospy.Subscriber('cmd_key', String, self.save_key, queue_size = 1)
        self.image_sub = rospy.Subscriber('image', Image, self.save_image, queue_size = 1)

    def SaveLine(self, string):
        string = string + '\n'
        with open('command_keys.txt', 'a') as out:
            out.write(string)

    def save_key(self, command):
        cadena = str(command) + str('\n')
        with open('command_keys.txt', 'a') as out:
            out.write(cadena)

    def save_image(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")   
        cv.imwrite(PATH + "_frame_" + str(self.frame) + ".jpg", cv_image)   
        self.frame += 1

    def Gather(self, command):
        data = rospy.wait_for_message("image", Image)
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")            
        cv.imshow("view", cv_image)
        cv.imwrite(PATH + "frame_" + str(self.frame) + "_" + command + ".jpg", cv_image)   
        self.frame += 1
        SaveLine()


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