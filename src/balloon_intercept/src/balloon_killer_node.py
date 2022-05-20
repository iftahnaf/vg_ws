#!/usr/bin/python3
import rospy
from cv_bridge import CvBridge
import threading
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMavFrame
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from copy import deepcopy
from balloon_trajectory import Balloon
import time
import numpy as np
from scipy.spatial.transform import Rotation as R


# rosservice call /mavros/setpoint_velocity/mav_frame "mav_frame: 8"

class BalloonKiller(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.rate = rospy.Rate(8)
        self.bridge = CvBridge()

        self.width = 320
        self.height = 240
        self.fov = 1.04719 #[rad]

        self.r_m = 0.5  #balloon radius in [m]
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

        self.center_sub = rospy.Subscriber('/balloon/center', Point, self.center_cb)
        self.radius_sub = rospy.Subscriber('/balloon/radius', Float32, self.radius_cb)
        self.center = []

    def center_cb(self, msg):
        self.center = [0, 0]
        self.center[0] = msg.x
        self.center[1] = msg.y
    
    def radius_cb(self, msg):
        self.radius = msg.data

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

    def range_est(self):
        ##range in drone NED system 
        x_range_cam = (self.width*self.r_m)/(2*self.radius*np.tan((self.fov)/2))
        y_range_cam = (self.center[0] - (self.width / 2.0))*self.r_m/self.radius
        z_range_cam = (self.center[1] - (self.height / 2.0))*self.r_m/self.radius
        
        range_cam = np.array([x_range_cam, y_range_cam, z_range_cam])
        
        return range_cam

    def create_vg_state(self):

            range_cam = self.range_est()
            local_range =  self.cam_2_local(range_cam)
            print(local_range)

            vpx = self.vel.twist.linear.x
            vpy = self.vel.twist.linear.y
            vpz = self.vel.twist.linear.z

            r = -local_range
            v = -np.array([vpx, vpy, vpz])
            print("v= {}".format(v))
            return r, v

    def cam_2_local(self, vec):
        ##rotate vector from cam to local (wgazebo world) system
        
        cam_2_drone_rot_mat = np.eye(3)  #rotation matrix from camera to drone system
        
        qx = self.pose.pose.orientation.x
        qy = self.pose.pose.orientation.y
        qz = self.pose.pose.orientation.z
        qw = self.pose.pose.orientation.w

        local_2_drone_rot_mat = R.from_quat([qx, qy, qz, qw]) #rotation matrix from local to drone system
        local_2_drone_rot_mat=local_2_drone_rot_mat.as_dcm()
        
        drone_2_local_rot_mat = np.linalg.inv(local_2_drone_rot_mat) #rotation matrix from drone to local system
        
        tmp_mat = cam_2_drone_rot_mat*drone_2_local_rot_mat*np.transpose(vec)
        
        rot_vec=np.array([tmp_mat[0][0], tmp_mat[1][1], tmp_mat[2][2]])

        return rot_vec

    def tgo_bounded(self, r, v, rho_u, rho_v, m=0.0, min_tgo=0.001):
        #define the variables of the polynom
        drho = rho_u - rho_v
        rr = np.dot(r,r)
        rv = np.dot(r,v)
        vv = np.dot(r,v)
        #define the polynom
        a0 = -rr+m**2
        a1 = -2.0*rv
        a2 = -vv +m*drho
        a3 = 0.0
        a4 = 0.25*drho**2
        P = np.poly1d([a4, a3, a2, a1, a0])
        # calculate Tgo
        sol = np.roots(P)
        real_sol = np.real(sol)[abs(np.imag(sol)) < 1e-5]
        real_sol = np.real(real_sol)[np.real(real_sol) > 0]
        if len(real_sol) > 1:
            Tgo = min(real_sol)
        elif len(real_sol) == 0:
            Tgo = min_tgo
        else:
            Tgo = real_sol
        return Tgo + min_tgo


    def cont_bounded(self, Tgo, r, v, g, max_thrust=1):

        ux_unb = ((r[0] + Tgo*v[0])/np.linalg.norm(r + Tgo*v))
        uy_unb = ((r[1] + Tgo*v[1])/np.linalg.norm(r + Tgo*v))
        uz_unb = ((r[2] + Tgo*v[2])/np.linalg.norm(r + Tgo*v))
        
        u = [ux_unb, uy_unb, uz_unb] * max_thrust

        return (u[0], u[1], u[2])

    def positioning(self):

        rospy.loginfo("***** Tracking the Balloon *****")
        while not rospy.is_shutdown():
            if self.center is not None:
                try:
                    rospy.loginfo("***** Sending positioning commands *****")
                    # self.center_frame_controller(0.01, 0.5, 0.0)
                    rospy.loginfo_throttle(1, "Balloon X: {}, Balloon Y: {}".format(self.center[0], self.center[1]))

                    if (self.width / 2.0) - self.center[0] < 2.0 and (self.height / 2.0) - self.center[1] < 2.0:
                        rospy.loginfo("Balloon is in the Middle of the Frame")
                        # self.close_distance()
                        
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
                self.create_vg_state()
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
