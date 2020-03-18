import rospy
import cv2 as cv
import sys
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import yaml

PATH_SYNC = os.path.abspath('images_stereo')
print('Saving path = {}'.format(PATH))
PATH_
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
        height, width, channel = 640, 480, 3
    
    def load_params():
        #Camera matrix
        self.Kl = [ [1086.56956,     0.     ,   359.39663],
             [0.     ,   830.34232,   227.63702],
             [0.     ,     0.     ,     1.     ]]
        #Distortion
        self.distl = [0.095862, 1.267249, 0.002377, 0.000967, 0.000000]
        
        self.Kr = [ [1029.15929,     0.     ,   409.73459],
             [0.     ,   779.47028,   291.03151],
             [0.     ,     0.     ,     1.     ]]
        self.distr = [0.159904, 0.099552, 0.045093, 0.026736, 0.000000]

        newcameramtxL, roiL = cv.getOptimalNewCameraMatrix(self.Kl, self.distl, (width, height), 1, (width,height))
        newcameramtxR, roiR = cv.getOptimalNewCameraMatrix(self.Kr, self.distr, (width, height), 1, (width,height))

    #Output cloud to rviz or somewhere
    def output(verts, colors):
        verts = verts.reshape(-1,3)
        colors = colors.reshape(-1,3)


    #Search for K, R and t optimised
    def sync(self, imageL, imageR):
        #Downscale images via pyramids
        #imageL = cv.pyrDown(self.bridge.imgmsg_to_cv2(imageL, "bgr8"))
        #imageR = cv.pyrDown(self.bridge.imgmsg_to_cv2(imageR, "bgr8"))
        
        #Rectify images with params of camera matrix(K) and distortion for each side of camera
        imageL = cv.undistort(imageL, self.Kl, self.distl, None, newcameramtxL)
        imageR = cv.undistort(imageR, self.Kr, self.distr, None, newcameramtxR)
        #Calculate disparity image
        stereo = cv.createStereoBM(numDisparities=16, blockSize=15)
        disparity = stereo.compute(imageL, imageR)
    
        f = 0.8 * width

        Q = np.float32([[1, 0, 0, -0.5*width],
                [0,-1, 0,  0.5*height], # turn points 180 deg around x-axis,
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
        cv.imshow('left', imgL)
        cv.imshow('disparity', (disp-min_disp)/num_disp)
        self.output(out_points, out_colors)
if __name__ == '__main__':
    rospy.init_node('StereoHandler', anonymous=True)
    handler = Stereo()    
#https://github.com/opencv/opencv/blob/master/samples/python/stereo_match.py




'''
# oST version 5.0 parameters


[image]

width
640

height
480

[narrow_stereo/left]

camera matrix
1086.569561 0.000000 359.396635
0.000000 830.342321 227.637024
0.000000 0.000000 1.000000

distortion
0.095862 1.267249 0.002377 0.000967 0.000000

rectification
0.792663 0.011737 0.609547
0.013634 0.999223 -0.036972
-0.609507 0.037617 0.791887

projection
930.009343 0.000000 -310.209211 0.000000
0.000000 930.009343 267.129143 0.000000
0.000000 0.000000 1.000000 0.000000
# oST version 5.0 parameters

[image]

width
640

height
480

[narrow_stereo/right]

camera matrix
1029.159285 0.000000 409.734592
0.000000 779.470284 291.031505
0.000000 0.000000 1.000000

distortion
0.159904 0.099552 0.045093 0.026736 0.000000

rectification
0.767144 -0.042989 0.640033
0.016130 0.998729 0.047749
-0.641272 -0.026306 0.766863

projection
930.009343 0.000000 -310.209211 -254.223585
0.000000 930.009343 267.129143 0.000000
0.000000 0.000000 1.000000 0.000000
'''