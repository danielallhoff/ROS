#!/usr/bin/env python

import rospy
import numpy as np
import cv2 as cv
import sys
import os
import std_msgs.msg
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point32
from cv_bridge import CvBridge, CvBridgeError
#Markers for points visualization: https://answers.ros.org/question/231781/rviz-markers-with-rospy/
from sensor_msgs.msg import PointCloud
import message_filters

PATH_SYNC = os.path.abspath('images_stereo')
print('Saving path = {}'.format(PATH_SYNC))

#Right: /robot1/trasera1
#Left: /robot1/trasera2
#Ros calibration node: http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration
class Stereo:
    def __init__(self):
        self.sub_trasR = message_filters.Subscriber("/robot1/trasera1/trasera1/rgb/image_raw", Image)
        self.sub_trasL = message_filters.Subscriber("/robot1/trasera2/trasera2/rgb/image_raw", Image)
        self.timeSync = message_filters.ApproximateTimeSynchronizer([self.sub_trasL, self.sub_trasR], queue_size=10,slop=0.2, allow_headerless=True)
        self.timeSync.registerCallback(self.sync)
        self.height, self.width, self.channel = 640, 480, 3
        self.load_params()
        self.bridge = CvBridge()
        self.stereo_cloud_publisher = rospy.Publisher("robot1/stereo_cloud", PointCloud, queue_size=10)
    
    def load_params(self):
        #Camera matrix
        self.Kl = np.float32([ [1086.56956,     0.     ,   359.39663],
             [0.     ,   830.34232,   227.63702],
             [0.     ,     0.     ,     1.     ]])
        #Distortion
        self.distl = np.float32([0.095862, 1.267249, 0.002377, 0.000967, 0.000000])
        
        self.Kr = np.float32([ [1029.15929,     0.     ,   409.73459],
             [0.     ,   779.47028,   291.03151],
             [0.     ,     0.     ,     1.     ]])
        self.distr = np.float32([0.159904, 0.099552, 0.045093, 0.026736, 0.000000])

        self.newcameramtxL, self.roiL = cv.getOptimalNewCameraMatrix(self.Kl, self.distl, (self.width, self.height), 1, (self.width, self.height))
        self.newcameramtxR, self.roiR = cv.getOptimalNewCameraMatrix(self.Kr, self.distr, (self.width, self.height), 1, (self.width, self.height))

    #Output cloud to rviz or somewhere
    def output(self, verts, colors):
        pointcloud = PointCloud()
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        #Use frame_id of the sensor
        header.frame_id = "map"
        pointcloud.header = header

        verts = verts.reshape(-1,3)
        #colors = colors.reshape(-1,3)
        
        for i in range(0,len(verts)):
            point = Point32()
            point.x = verts[i][0]
            point.y = verts[i][1]
            point.z = verts[i][2]
            pointcloud.points.append(point)
        self.stereo_cloud_publisher.publish(pointcloud)


    #Search for K, R and t optimised
    def sync(self, imageL, imageR):
        #Downscale images via pyramids
        #imageL = cv.pyrDown(self.bridge.imgmsg_to_cv2(imageL, "bgr8"))
        #imageR = cv.pyrDown(self.bridge.imgmsg_to_cv2(imageR, "bgr8"))

        matL = self.bridge.imgmsg_to_cv2(imageL, desired_encoding='mono8')
        arrayL = np.asarray(matL)

        matR = self.bridge.imgmsg_to_cv2(imageR, desired_encoding='mono8')
        arrayR = np.asarray(matR)

        #Rectify images with params of camera matrix(K) and distortion for each side of camera
        imageL = cv.undistort(arrayL, self.Kl, self.distl, None, self.newcameramtxL)
        imageR = cv.undistort(arrayR, self.Kr, self.distr, None, self.newcameramtxR)

        #Calculate disparity image
        stereo = cv.StereoBM_create(numDisparities=16, blockSize=15)
        disparity = stereo.compute(imageL, imageR)
    
        f = 0.8 * self.width

        Q = np.float32([[1, 0, 0, -0.5*self.width],
                [0,-1, 0,  0.5*self.height], # turn points 180 deg around x-axis,
                [0, 0, 0,     -f], # so that y-axis looks up
                [0, 0, 1,      0]])
        #Obtain points
        points3d = cv.reprojectImageTo3D(disparity, Q)
        #Obtain colors
        #colors3d = cv.cvtColor(imageL, cv.COLOR_BGR2RGB)
        #Apply mask
        mask = disparity > disparity.min()
        out_points = points3d[mask]
        #out_colors = colors3d[mask]
        out_colors = []
        #cv.imshow('left', imageL)
        #cv.imshow('disparity', (disparity-min_disp)/num_disp)
        self.output(out_points, out_colors)

if __name__ == '__main__':
    rospy.init_node('StereoHandler', anonymous=True)
    handler = Stereo()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

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