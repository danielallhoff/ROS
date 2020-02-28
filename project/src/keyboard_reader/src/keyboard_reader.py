#!/usr/bin/env python
#http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rospy
import cv2 as cv
import sys
from threading import Event, Thread
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

command_message = """
w/s : increase/decrease only linear speed by 10%
a/d : increase/decrease only angular speed by 10%

. to quit
"""

def call_repeatedly(interval, func, *args):
        stopped = Event()
        def loop():
            while not stopped.wait(interval): # the first call is in "interval" secs
                func(*args)
        Thread(target=loop).start()    
        return stopped.set

class KeyBoardHandler:
    def __init__(self):
        
        pubDest = '/robot1/mobile_base/commands/velocity'
        #pubDest = 'cmd_vel'
        self.cmd_vel_pub = rospy.Publisher(pubDest, Twist, queue_size=1)
        self.frame = 0
        self.cmd_key_pub = rospy.Publisher('cmd_key', String, queue_size=1)
        self.last_key = 'n'

    def driveKeyboard(self):
        twist = Twist()
        twist.linear.x = 0.5
        #cancel_future_calls = call_repeatedly(1, self.Timeout)
        while(1):
            cmd_line = raw_input(command_message)
            if cmd_line:
                char = cmd_line[0]
                self.last_key = char
            else:
                char = self.last_key

            if((char != 'w') and (char != 'a') and (char != 's') and (char != 'd') and (char != '.')):
                char = 'n'

            # Salida red -> 0
            if char == 'w':
                twist.angular.z = 0
                twist.linear.x = twist.linear.x * 1.1
            # Salida red -> 1
            elif char == 'a':
                twist.angular.z = 0.15
                twist.linear.x = twist.linear.x * 0.8
            # Salida red -> 2
            elif char == 's':
                twist.angular.z = 0
                twist.linear.x = twist.linear.x * 0.9
            # Salida red -> 3
            elif char == 'd':
                twist.angular.z = -0.15
                twist.linear.x = twist.linear.x * 0.8
            # Exit
            elif char == '.':
                #cancel_future_calls()
                break
                            
            self.cmd_vel_pub.publish(twist)

    def Timeout(self):
        self.cmd_key_pub.publish(self.last_key)


def main():
    rospy.init_node('KeyBoardHandler', anonymous=True)
    handler = KeyBoardHandler()
    handler.driveKeyboard()
   
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)