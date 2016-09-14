#!/usr/bin/env python
# import roslib
# roslib.load_manifest('~/ros-workspaces/hackathon_ws/src/hackathon_project/package.xml')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2",Image)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)

    def callback(self,data):
        try:
            imgOriginal = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print("==[CAMERA MANAGER]==", e)
        # (rows,cols,channels) = imgOriginal.shape
        # if cols > 60 and rows > 60:
        #     cv2.circle(imgOriginal,(50,50),10,255)

        imgGrayscale = cv2.cvtColor(imgOriginal, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(imgGrayscale,cv2.cv.CV_HOUGH_GRADIENT,1.2,100)
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x,y,r) in circles:
                cv2.circle(imgGrayscale, (x, y), r, (0, 255, 0), 4)

        hsv = cv2.cvtColor(imgOriginal, cv2.COLOR_BGR2HSV)

        lower = np.array([0,110,110])
        upper = np.array([50,255,255])
    	mask = cv2.inRange(hsv, lower, upper)
        output = cv2.bitwise_and(imgOriginal, imgOriginal, mask = mask)

        # imgGrayscale = cv2.cvtColor(imgOriginal, cv2.COLOR_BGR2GRAY)
        # imgBlurred = cv2.GaussianBlur(imgGrayscale, (5, 5), 0)
        # imgCanny = cv2.Canny(imgBlurred, 100, 200)

        cv2.imshow("GrayScale Window", imgGrayscale)
        cv2.imshow("Image Window", output)
        # cv2.imshow("Blurred Window", imgBlurred)
        # cv2.imshow("CannyEdges Window", imgCanny)
        cv2.waitKey(3)

def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
        #print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
