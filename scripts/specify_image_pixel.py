#!/usr/bin/env python
import rospy
import rospkg
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2


class SpecifyPixelNode:

    def __init__(self):
        rospy.init_node('SpecifyPixel', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("kinect_head/qhd/image_color", Image, self.callback, queue_size=1, buff_size=2**24)
        self.pixel_pub = rospy.Publisher('image_pixel', Int32MultiArray, queue_size=1)
        rospy.loginfo("Initialized SpecifyPixelNode.")
        self.window_name = "image"
        self.pixel_point = Int32MultiArray()
        self.pixel_point.data = [0,0]
        self.offset = 5
        self.u = 480 # qhd 960/2
        self.v = 270 # qhd 540/2

    def mouseParam(self, window_name):
        self.mouseEvent = {"x":None, "y":None, "event":None, "flags":None}
        cv2.setMouseCallback(window_name, self.__CallBackFunc, None)    

    def getPos(self):
        return (self.mouseEvent["x"], self.mouseEvent["y"])

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.circle(cv_image, (self.u, self.v), 15, (0, 165, 255), -1)
            cv2.imshow(self.window_name, cv_image)
            key = cv2.waitKey(3)
            if key & 0xFF == ord("d"):
                self.u += self.offset
            if key & 0xFF == ord("a"):
                self.u -= self.offset
            if key & 0xFF == ord("w"):
                self.v -= self.offset
            if key & 0xFF == ord("s"):
                self.v += self.offset
                
            self.pixel_point.data[0] = self.u
            self.pixel_point.data[1] = self.v

            self.pixel_pub.publish(self.pixel_point)
 
        except CvBridgeError as e:
            print(e)


def main():
    SpecifyPixelNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
         print("Shutting down")

if __name__ == '__main__':
    main()
