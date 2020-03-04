import rospy
import cv2 as cv
import sys
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


PATH_SYNC = os.path.abspath('images_stereo')
print('Saving path = {}'.format(PATH))
#Right: /robot1/trasera1
#Left: /robot1/trasera2
#Ros calibration node: http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration
class Stereo:
    __init__(self):
        sub_tras1 = rospy.Subscriber("/robot1/trasera1/trasera1/rbg/image_raw")
        sub_tras2 = rospy.Subscriber("/robot1/trasera2/trasera2/rbg/image_raw")

    __init__(self):

    #Search for K, R and t optimised
    def sync(self):

