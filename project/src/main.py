#http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rospy
import cv2 as cv
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from keras.models import load_model

###########
#CONSTANTS#
###########
batch_size = 64
nb_classes = 4
epochs = 20
img_rows, img_cols = 32, 32

class TurtleBotHandler:
    def __init__(self):
        self.bridge = CvBridge()
        
        self.model_movement = 
        
        #Subscribe to camera of robot and receive data
        self.image_sub = rospy.Subscriber("robot1/camera/rgb/image_raw", Image, self.callback,  queue_size=1)
        self.image_pub = rospy.Publisher('/robot1/mobile_base/commands/velocity', Twist, queue_size = 1)
        #self.camera1_sub = rospy.Subscriber("")
        #self.camera2_sub = 
    def __init__(self, model):
        self.bridge = CvBridge()
        
        self.model_movement = model
        
        #Subscribe to camera of robot and receive data
        self.image_sub = rospy.Subscriber("robot1/camera/rgb/image_raw", Image, self.callback,  queue_size=1)
        self.image_pub = rospy.Publisher('/robot1/mobile_base/commands/velocity', Twist, queue_size = 1)
    #Each image process
    def callback(self, data):
        twist = Twist()
        #Obtain images
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv.imshow("view", cv_image)
            cv.waitKey(30)
        except Exception as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        #Image size to big downsize
        if cols > 60 and rows > 60:
            cv.circle(cv_image, (50,50), 10, 255)

        #Apply movement
        pred = model_movement.predict(cv_image)
        
        num_class = np.argmax(pred, axis = 1)

        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        if num_class == 0:
            twist.linear.x = 0.50
        elif num_class == 1:
            twist.angular.z = 0.75
            twist.linear.x = 0.25
        elif num_class == 2:
            twist.angular.z = -0.75
            twist.linear.x = 0.25
        elif num_class == 3:
            twist.linear.x = -0.50
        else:
            print("DO NOTHING")

        #cv.imshow("Robot detector", cv_image_with_detector)
        
        #Send prediction
        image_pub.publish(twist)


def main(args):
    #Load turtlebothandler
    #model = load_model(FILEPATH)
    handler = TurtleBotHandler(model)
    rospy.init_node('TurtleBotHandler', anonymous=True)
    rate = rospy.Rate(30)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)