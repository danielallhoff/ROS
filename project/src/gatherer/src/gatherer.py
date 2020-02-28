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
PATH = PATH + "/images"
print('Saving path = {}'.format(PATH))

class Gatherer:
    def __init__(self):
        self.frame = 0
        self.bridge = CvBridge()
        self.dir_image_sub = message_filters.Subscriber('/robot1/camera/rgb/image_raw', Image)
        self.vel_sub = message_filters.Subscriber('/robot1/mobile_base/commands/velocity', Twist)
        self.timeSync = message_filters.ApproximateTimeSynchronizer([self.dir_image_sub, self.vel_sub], queue_size=10,slop=0.1, allow_headerless=True)
        self.timeSync.registerCallback(self.SyncCallback)
        #self.cmd_sub = rospy.Subscriber('cmd_key', String, self.save_key, queue_size = 1)
        #self.image_sub = rospy.Subscriber('image', Image, self.save_image, queue_size = 1)
        self.angular_velocity = 0
        self.linear_velocity = 0

    def SyncCallback(self, image, velocity):
        self.save_velocity(velocity)
        self.save_image(image)


    def save_velocity(self, twist):
        '''Saves velocities linear and angular into vel.txt'''
        cadena = str(twist.linear.x) + ';' + str(twist.angular.z) + ';'
        with open('vel.txt', 'a') as out:
            out.write(cadena + str('\n'))

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv.imwrite(PATH + "_frame_" + str(self.frame) + ".jpg", cv_image)   
        self.frame += 1
        self.save_velocity()

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
