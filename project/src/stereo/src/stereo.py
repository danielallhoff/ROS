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
        self.sub_trasR = rospy.Subscriber("/robot1/trasera1/trasera1/rbg/image_raw")
        self.sub_trasL = rospy.Subscriber("/robot1/trasera2/trasera2/rbg/image_raw")
        self.timeSync = message_filters.ApproximateTimeSynchronizer([self.sub_trasL, self.sub_trasR], queue_size=10,slop=0.1, allow_headerless=True)
        self.timeSync.registerCallback(self.sync)
        self.load_params()
        self.bridge = CvBridge()
    
    def load_params():
        R = []
        D = []
        K = []
        R = []
        P = []

    #Search for K, R and t optimised
    def sync(self, imageL, imageR):
        #Downscale images via pyramids
        imageL = cv.pyrDown(self.bridge.imgmsg_to_cv2(imageL, "bgr8"))
        imageR = cv.pyrDown(self.bridge.imgmsg_to_cv2(imageR, "bgr8"))
        #Calculate disparity image
        stereo = cv.createStereoBM(numDisparities=16, blockSize=15)
        disparity = stereo.compute(imageL, imageR)
        Q = np.float32([[1, 0, 0, -0.5*w],
                [0,-1, 0,  0.5*h], # turn points 180 deg around x-axis,
                [0, 0, 0,     -f], # so that y-axis looks up
                [0, 0, 1,      0]])
        #Obtain points
        points3d = cv.reprojectImageTo3D(disparity, Q)
        #Obtain colors
        colors3d = cv.cvtColor(imgL, cv.COLOR_BGR2RGB)
        #Apply mask
        mask = disp > disp.min()
        out_points = points[mask]
        out_colors = colors[mask]

#https://github.com/opencv/opencv/blob/master/samples/python/stereo_match.py

        #Create cloud



