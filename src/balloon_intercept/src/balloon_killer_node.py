#!/usr/bin/python3
import rospy
from cv_bridge import CvBridge
import gazebo_balloon_detector
import threading
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMavFrame
from copy import deepcopy
from balloon_trajectory import Balloon
import time
import numpy as np

# rosservice call /mavros/setpoint_velocity/mav_frame "mav_frame: 8"

class BalloonKiller(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.rate = rospy.Rate(8)
        self.bridge = CvBridge()

        self.video = gazebo_balloon_detector.Video(self)
        self.video.start()
        self.width = 320
        self.height = 240

        self.counter = 0

        self.pose = PoseStamped()
        self.vel = TwistStamped()
        self.state = State()

        self.des_vel = TwistStamped()
        self.des_vel.twist.angular.x = 0
        self.des_vel.twist.angular.y = 0
        self.des_vel.twist.linear.y = 0

        self.pose_sub = rospy.Subscriber(
            '/mavros/local_position/pose', PoseStamped, self.pose_cb)
        self.vel_sub = rospy.Subscriber(
            '/mavros/local_position/velocity_local', TwistStamped, self.vel_cb)
        self.state_sub = rospy.Subscriber(
            '/mavros/state', State, self.state_cb)

        self.vel_pub = rospy.Publisher(
            '/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
        self.pose_pub = rospy.Publisher(
            '/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        self.balloon = Balloon()

        self.change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.vel_frame = rospy.ServiceProxy('/mavros/setpoint_velocity/mav_frame', SetMavFrame)

    def __call__(self, center, radius):
        self.center = center
        self.radius = radius

    def pose_cb(self, msg):
        self.pose = deepcopy(msg)

    def vel_cb(self, msg):
        self.vel = deepcopy(msg)
    
    def state_cb(self, msg):
        self.state = deepcopy(msg)

    def takoff_seq(self, des_pose):
        if self.state.armed == 0:
            rospy.wait_for_service('/mavros/cmd/arming')
            try:
                self.arming(True)
                rospy.loginfo("***** Arming ******")
            except rospy.ServiceException as e:
                print(e)
        
        if self.state.mode not in 'OFFBOARD':
            rospy.wait_for_service('/mavros/set_mode')
            try:
                k = 0
                while k < 10:
                    self.pose_pub.publish(des_pose)
                    self.rate.sleep()
                    k += 1
                rospy.loginfo("***** Changing to OFFBOARD Mode ******")
                self.change_mode(custom_mode='OFFBOARD')
            except rospy.ServiceException as e:
                print(e)

        rospy.wait_for_service('/mavros/setpoint_velocity/mav_frame')
        try:
            self.vel_frame(mav_frame=8)
        except rospy.ServiceException as e:
            print(e)
        
    def initialPose(self):
        des_pose = PoseStamped()
        err = 0.5
        des_pose.pose.position.x = 0.0
        des_pose.pose.position.y = 0.0
        des_pose.pose.position.z = 5.0

        self.takoff_seq(des_pose)

        while not rospy.is_shutdown():
            z = self.pose.pose.position.z
            self.pose_pub.publish(des_pose)
            if des_pose.pose.position.z - z < err:
                break
            self.rate.sleep()
        
    def scanning(self):
        self.initialPose()
        while not rospy.is_shutdown():
            if self.center is not None:
                rospy.loginfo("***** Found Balloon! *****")
                self.balloon = deepcopy(self.pose)
                self.counter = 0
                rospy.loginfo("Balloon Center: ({},{})".format(
                    self.center[0], self.center[1]))
                self.hold()
                break

            else:
                self.des_vel.twist.linear.x = 0.0
                self.des_vel.twist.linear.z = 0.0
                self.des_vel.twist.angular.z = 0.2
                self.vel_pub.publish(self.des_vel)

            rospy.loginfo_throttle(10, "***** Scanning for Balloon *****")
            self.rate.sleep()

    def hold(self):
        hold_time = 2
        hold_time_start = time.time()
        while time.time() < hold_time + hold_time_start:
            self.pose_pub.publish(self.balloon)
            self.rate.sleep()

    def center_frame_controller(self, kw, kh, kr):
        des_range = 2.0
        range = self.balloon.est_range()
        if range:
            rospy.loginfo(" Range from balloon = {}".format(range))
        else:
            rospy.loginfo("Can't calculate range...")
        self.des_vel.twist.angular.z = kw * ((self.width / 2.0) - self.center[0])
        self.des_vel.twist.linear.z = kh * ((self.height / 2.0) - self.center[1])
        self.des_vel.twist.linear.x = - kr *((des_range - range))

        if self.des_vel.twist.linear.x > 5.0:
            self.des_vel.twist.linear.x = 5.0

        self.vel_pub.publish(self.des_vel)

    def positioning(self):

        rospy.loginfo("***** Tracking the Balloon *****")
        while not rospy.is_shutdown():
            if self.center is not None:
                try:
                    rospy.loginfo("***** Sending positioning commands *****")
                    self.center_frame_controller(0.01, 0.5, 0.0)
                    rospy.loginfo_throttle(1, "Balloon X: {}, Balloon Y: {}".format(self.center[0], self.center[1]))

                    if (self.width / 2.0) - self.center[0] < 2.0 and (self.height / 2.0) - self.center[1] < 2.0:
                        rospy.loginfo("Balloon is in the Middle of the Frame")
                        self.close_distance()
                        
                    if self.pose.pose.position.z < 1.0:
                        self.des_vel.twist.linear.z = 0.0
                except :
                    rospy.loginfo("***** Failed to send positioning commands *****")
                    continue
            else:
                rospy.loginfo("Ballon is dissapeared from frame...") 
                self.counter += 1
                self.pose_pub.publish(self.pose)
                if self.counter > 50:
                    rospy.loginfo("Ballon is gone, restarting...") 
                    time.sleep(10)
                    break
            self.rate.sleep()

    def close_distance(self):
        rospy.loginfo("****** Start Catching Balloon *****")
        while not rospy.is_shutdown():
            if self.center is None:
                rospy.loginfo("Balloon Out of the Frame, Back to Scanning")
                break
            self.center_frame_controller(0.001, 0.005, 0.3)
            self.rate.sleep()

    def run(self):
        self.initialPose()
        while not rospy.is_shutdown():
            try:
                self.counter = 0
                self.center = None
                self.scanning()
                self.positioning()
            except KeyboardInterrupt:
                rospy.signal_shutdown("Done")
                break

def main():
    rospy.init_node('balloon_killer', anonymous=True, disable_signals=True)
    rospy.loginfo_once('***** Initiate Balloon Killer Node *****')
    drone = BalloonKiller()
    drone.start()
    drone.join()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
        pass
