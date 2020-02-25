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

class KeyBoardHandler:
    def __init__(self):
        
        pubDest = '/robot1/mobile_base/commands/velocity'
        #pubDest = 'cmd_vel'
        self.cmd_vel_pub = rospy.Publisher(pubDest,Twist, queue_size=1)
        self.frame = 0
        self.cmd_key_pub = rospy.Publisher('cmd_key',String, queue_size=1)
        self.last_key = ' '

    def driveKeyboard(self):
        twist = Twist()
        cancel_future_calls = self.call_repeatedly(1, self.Timeout)
        print('Before loop')
        while(1):
            print('Another loop iteration')
            cmd_line = raw_input("Type a command and then press enter. Use '+' to move forward, 'm' move backward, 'l' to turn left, 'r' to turn right, '.' exit ")
            if cmd_line:
                char = cmd_line[0]
                self.last_key = char
                if((char != '+') and (char != 'l') and (char != 'r') and (char != '.') and (char != 'm')):
                    print("Unknown command " + cmd_line)
                    self.last_key = ' '

                twist.linear.x = 0
                twist.linear.y = 0
                twist.linear.z = 0
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = 0

                # Salida red -> 0
                if char == '+':
                    twist.linear.x = 0.50
                # Salida red -> 1
                elif char == 'l':
                    twist.angular.z = 0.75
                    twist.linear.x = 0.25
                # Salida red -> 2
                elif char == 'r':
                    twist.angular.z = -0.75
                    twist.linear.x = 0.25
                # Exit
                elif char == '.':
                    cancel_future_calls()
                    break
                # Salida red -> 3
                elif char == 'm':
                    twist.linear.x = -0.50
                                
                self.cmd_vel_pub.publish(twist)

            else:
                print("Empty command")


    def call_repeatedly(self, interval, func, *args):
        stopped = Event()
        def loop():
            while not stopped.wait(interval): # the first call is in "interval" secs
                func(*args)
        Thread(target=loop).start()    
        return stopped.set
  
    def Timeout(self):
        self.cmd_key_pub.publish(self.last_key)
        self.last_key = ' '
        print('Timeout')


def main():
    rospy.init_node('KeyBoardHandler', anonymous=True)
    handler = KeyBoardHandler()
    handler.driveKeyboard()
   
if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)