#!/usr/bin/env python
#http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rospy
import cv2 as cv
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import String
class KeyBoardHandler:
    def __init__(self, nh):
        self.nh = nh
        cmd_vel_pub = rospy.Publisher('/robot1/mobile_base/commands/velocity',Twist, queue_size=1)

    def driveKeyboard(self):

        twist = Twist()

        while(nh.ok()):
            cmd_line = raw_input("Type a command and then press enter. Use '+' to move forward, 'l' to turn left, 'r' to turn right, '.' exit ")
            char = cmd_line[0]
            if(char != '+' & char != 'l' & char != 'r' & char != '.'):
                print("Unknown command " + cmd_line)
            twist.linear.x = 0
            twist.linear.y = 0
            twist.angular.z = 0
            if char == '+':
                twist.linear.x = 0.25
            elif char == 'l':
                twist.angular.z = 0.75
                twist.linear.x = 0.25
            elif char == 'r':
                twist.angular.z = -0.75
                twist.linear.x = 0.25
            elif char == '.':
                break
            
            cmd_vel_pub.publish(twist)
        
        

def main():    
    rospy.init_node('KeyBoardHandler', anonymous=True)
    handler = KeyBoardHandler()
    handler.driveKeyboard()
   
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)