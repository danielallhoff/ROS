#!/usr/bin/env python
#http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rospy
import cv2 as cv
import sys
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

PATH = os.path.abspath('images')
print('Saving path = {}'.format(PATH))

class ImageReader:
    def __init__(self):
        self.bridge = CvBridge()
        #Send cv images back
        #self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=5)
        #Subscribe to camera of robot and receive data
        self.image_sub = rospy.Subscriber('/robot1/camera/rgb/image_raw', Image, self.callback, queue_size = 1)
        self.image_pub = rospy.Publisher('image', Image)
        print("Init")

    def callback(self, data):
        #Obtain images
        print("Callback call")
        try:
            #For receive key            
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")            
            cv.imshow("view", cv_image)
            #cv.imwrite(PATH + "frame_" + str(self.frame) + "_" + msg + ".jpg", cv_image)
            
            cv.waitKey(30)
        except Exception as e:
            print(e)
        print("Image sent")
        (rows, cols, channels) = cv_image.shape
        #Image size to big downsize
        if cols > 60 and rows > 60:
            cv.circle(cv_image, (50,50), 10, 255)

        cv.imshow("Image window", cv_image)
        cv.waitKey(3)
        image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        image_pub.publish(image)
def main():
    print("Start")
    #Load ImageReader
    handler = ImageReader()
    
    rospy.init_node('ImageReader', anonymous=True)
    rate = rospy.Rate(2)
    try:
        print("Spin")
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv.destroyAllWindows()

if __name__ == '__main__':
    main()