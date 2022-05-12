#!/usr/bin/python3

import cv2
import numpy as np
import imutils
import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
import queue

class ROSBalloon():
    def __init__(self):
        self.rate = rospy.Rate(60)
        self.bridge = CvBridge()
        self.frame_sub = rospy.Subscriber('/iris/usb_cam/image_raw', Image, self.image_cb)
        self.center_pub = rospy.Publisher('/balloon/center', Point, queue_size=10)
        self.radius_pub = rospy.Publisher('/balloon/radius', Float32, queue_size=10)
        self.frame_buffer = queue.Queue()
        self.center = Point()
        self.radius = Float32()

    def image_cb(self, msg):
        self.frame_buffer.put(self.bridge.imgmsg_to_cv2(msg))

    def findBalloon(self, camera_width):
            greenLower = (29, 86, 6)
            greenUpper = (64, 255, 255)

            if not self.frame_buffer.empty():
 
                self.frame = self.frame_buffer.get()
                self.frame = imutils.resize(self.frame, width=camera_width)
                _blurred = cv2.GaussianBlur(self.frame, (11, 11), 0)
                _hsv = cv2.cvtColor(_blurred, cv2.COLOR_BGR2HSV)

                _mask = cv2.inRange(_hsv, greenLower, greenUpper)
                _mask = cv2.erode(_mask, None, iterations=2)
                _mask = cv2.dilate(_mask, None, iterations=2)

                _cnts = cv2.findContours(_mask.copy(), cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
                _cnts = imutils.grab_contours(_cnts)
                center = None

                if len(_cnts) > 0:
                    _c = max(_cnts, key=cv2.contourArea)
                    ((x, y), radius) = cv2.minEnclosingCircle(_c)
                    M = cv2.moments(_c)
                    center = (int(M["m10"] / M["m00"]),
                                int(M["m01"] / M["m00"]))
                    if radius > 0:
                        cv2.circle(self.frame, center,
                                5, (0, 0, 255), -1)
                try:
                    return center, radius
                except:
                    return None, None


    def run(self):
        while not rospy.is_shutdown():
            try:
                center, radius = self.findBalloon(320)
            except:
                center = None
                radius = None
                continue

            if center and radius:
                self.center.x = center[0]
                self.center.y = center[1]
                self.radius.data = radius
                self.center_pub.publish(self.center)
                self.radius_pub.publish(self.radius)

            cv2.imshow('frame', self.frame)

            if cv2.waitKey(10) & 0xFF == ord('q'):
                rospy.loginfo("Terminate balloon_detector node...")
                break
            self.rate.sleep()

def main():
    rospy.init_node('balloon_detector')
    rospy.loginfo("Starting balloon detector node ... ")
    ros_balloon = ROSBalloon()
    ros_balloon.run()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
        pass
