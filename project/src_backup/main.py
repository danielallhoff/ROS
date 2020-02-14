import rospy
import cv2 as cv
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class TurtleBotHandler:
    def __init__(self):
        self.bridge = CvBridge()

def callback(msg):
    try:
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv.imshow("view", cv_image)
        cv.waitKey(3)
    except exception:
        print(exception)

def main(args):
    handler = TurtleBotHandler()
    rospy.init_node('TurtleBotHandler', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv.destroyAllWindows()
if __name__ == '__main__':
    try:
        main(sys.args)