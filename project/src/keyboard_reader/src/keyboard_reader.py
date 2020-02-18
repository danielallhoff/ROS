#!/usr/bin/env python
#http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rospy
import cv2 as cv
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
class KeyBoardHandler:
    def __init__(self):
        #pubDest = '/robot1/mobile_base/commands/velocity'
        pubDest = 'cmd_vel'
        self.cmd_vel_pub = rospy.Publisher(pubDest,Twist, queue_size=1)
        publishCommand = 'cmd_key'
        self.cmd_key = rospy.Publisher(pubDest,String, queue_size=1)

    def driveKeyboard(self):
        twist = Twist()
        while(1):
            cmd_line = raw_input("Type a command and then press enter. Use '+' to move forward, 'l' to turn left, 'r' to turn right, '.' exit ")
            char = cmd_line[0]
            if((char != '+') and (char != 'l') and (char != 'r') and (char != '.')):
                print("Unknown command " + cmd_line)

            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
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
            self.cmd_key.publish(char)
            self.cmd_vel_pub.publish(twist)
        
        

def main():    
    rospy.init_node('KeyBoardHandler', anonymous=True)
    handler = KeyBoardHandler()
    handler.driveKeyboard()
   
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)