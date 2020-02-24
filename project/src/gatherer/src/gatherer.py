#!/usr/bin/env python
import rospy
import cv2 as cv
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

PATH = os.path.abspath('images')
print('Saving path = {}'.format(PATH))

class Gatherer:
    def __init__(self):
        #self.image_sub = rospy.Subscriber('/robot1/camera/rgb/image_raw', Image, self.Gather, queue_size = 1, buff_size = 2**30)
        self.frame = 0
        self.bridge = CvBridge()
        self.cmd_sub = rospy.Subscriber('cmd_key', String, self.Gather, queue_size = 1)
        #self.image_sub = rospy.Subscriber('image', Image, queue_size = 1)

    def SaveLine(self, string):
        string = string + '\n'
        with open('command_keys.txt', 'a') as out:
            out.write(string)

    def Gather(self, command):
        data = rospy.wait_for_message("image", Image)
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")            
        cv.imshow("view", cv_image)
        cv.imwrite(PATH + "frame_" + str(self.frame) + "_" + command + ".jpg", cv_image)   
        frame += 1
        SaveLine()


def main():
    handler = Gatherer()
    rospy.init_node('Gatherer', anonymous=True)
    rate = rospy.Rate(2)
   
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)