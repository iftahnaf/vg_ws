#!/usr/bin/python3
from tkinter import Y
from turtle import end_fill
import rospy
from cv_bridge import CvBridge
import threading
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3Stamped
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMavFrame
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from gazebo_msgs.srv import GetModelState
from copy import deepcopy
from balloon_trajectory import Balloon
import time
import numpy as np
from scipy.spatial.transform import Rotation as R


# rosservice call /mavros/setpoint_velocity/mav_frame "mav_frame: 8"

class BalloonKiller(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.rate = rospy.Rate(20)
        self.bridge = CvBridge()

        self.width = 320
        self.height = 240
        self.fov = 1.04719 #[rad]

        self.target_position = np.array([0.0, 0.0, 0.0])  #save target position in local (world) frame. init to origin

        self.r_m = 0.5  #balloon radius in [m]
        self.counter = 0

        self.pose = PoseStamped()
        self.vel = TwistStamped()
        self.state = State()

        self.des_vel = TwistStamped()
        self.des_vel.twist.angular.x = 0
        self.des_vel.twist.angular.y = 0
        self.des_vel.twist.linear.y = 0

        self.rho_u = 18.0  
        self.rho_v = 9.81

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
        self.acc_pub = rospy.Publisher(
            '/mavros/setpoint_accel/accel', Vector3Stamped, queue_size=10)
        self.acc_cmd = Vector3Stamped()

        self.balloon = Balloon()

        self.change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.vel_frame = rospy.ServiceProxy('/mavros/setpoint_velocity/mav_frame', SetMavFrame)

        self.center_sub = rospy.Subscriber('/balloon/center', Point, self.center_cb)
        self.radius_sub = rospy.Subscriber('/balloon/radius', Float32, self.radius_cb)
        self.center = []
        self.tmp_center = self.center
        
        self.min_range_norm = 10000 #save minimal range norm - init to high value

        self.balloon_in_frame = True

        self.real_baloon_pose = None 

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
        z = self.pose.pose.position.z
        
        while des_pose.pose.position.z - z > err:
            z = self.pose.pose.position.z
            self.pose_pub.publish(des_pose)
            self.rate.sleep()
        
    def scanning(self):
        self.initialPose()
        
        balloon_in_range = False
        x_in_range = False
        y_in_range = False
        
        while not balloon_in_range:
            if self.center and self.radius:
                if self.center[0] > (2*self.radius) and self.center[0] < (self.width - 2*self.radius):
                    x_in_range = True
                
                if self.center[1] > (2*self.radius) and self.center[1] < (self.height - 2*self.radius):
                    y_in_range = True

                if x_in_range and y_in_range:
                    balloon_in_range = True
        
            self.des_vel.twist.linear.x = 0.0
            self.des_vel.twist.linear.z = 0.0
            self.des_vel.twist.angular.z = 0.2
            self.vel_pub.publish(self.des_vel)
            rospy.loginfo_throttle(10, "***** Scanning for Balloon *****")
            self.rate.sleep()

        rospy.loginfo("***** Found Balloon! *****")
        self.balloon = deepcopy(self.pose)
        self.counter = 0
        rospy.loginfo("Balloon Center: ({},{})".format(self.center[0], self.center[1]))
        self.hold()

    def hold(self):
        hold_time = 2
        hold_time_start = time.time()
        while time.time() < hold_time + hold_time_start:
            self.pose_pub.publish(self.balloon)
            self.rate.sleep()

    def range_est(self):
     
        ##range in drone ENU system 
        x_range_cam = (self.width*self.r_m)/(2*self.radius*np.tan((self.fov)/2))  
        y_range_cam = -(self.center[0] - (self.width / 2.0))*self.r_m/self.radius
        z_range_cam = -(self.center[1] - (self.height / 2.0))*self.r_m/self.radius

        #multiply by correction factors
        x_range_cam = x_range_cam*1.12

        range_cam = np.array([x_range_cam, y_range_cam, z_range_cam])
        
        return range_cam

    def create_vg_state(self, balloon_in_frame):
        rp_x = self.pose.pose.position.x
        rp_y = self.pose.pose.position.y
        rp_z = self.pose.pose.position.z
            
        rp = np.array([rp_x, rp_y, rp_z])  #drone position (from GPS)

        if  balloon_in_frame:
            print("balloon in frame")
            range_cam = self.range_est()
            local_range =  self.cam_2_local(range_cam)

            self.target_position = rp + local_range #while balloon in frame, update it's location in local frame
            print("cam_range = {}".format(range_cam))
            print("range = {}".format(local_range))
            print("target position = {}".format(self.target_position))
            r = local_range

        else:   #if baloon is out of frame, use drone GPS position and last known baloon position
            print("balloon out of frame")
            r = self.target_position - rp  
            
            print("range = {}".format(r))
            print("target position = {}".format(self.target_position))
        
        range_norm = np.linalg.norm(r)
        self.min_range_norm = min(self.min_range_norm, range_norm) #save the smallest norm
        rospy.loginfo("Min range norm = {}".format(self.min_range_norm))
        
        norm_to_real_balloon_pose = np.linalg.norm(self.real_baloon_pose-rp)  ##calculate eal miss distance
        rospy.loginfo("Norm to real baloon position = {}".format(norm_to_real_balloon_pose))


        vpx = self.vel.twist.linear.x  
        vpy = self.vel.twist.linear.y
        vpz = self.vel.twist.linear.z

        v = -np.array([vpx, vpy, vpz]) #balloon stationary, so relative vel = drone vel
        #print("v= {}".format(v))
        return r, v

    def cam_2_local(self, vec):
        ##rotate vector from cam to local (gazebo world) system
        
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


    def cont_bounded(self, Tgo, r, v, max_thrust=1):

        ux_unb = ((r[0] + Tgo*v[0])/np.linalg.norm(r + Tgo*v))
        uy_unb = ((r[1] + Tgo*v[1])/np.linalg.norm(r + Tgo*v))
        uz_unb = ((r[2] + Tgo*v[2])/np.linalg.norm(r + Tgo*v))
        
        u = np.array([ux_unb, uy_unb, uz_unb]) * max_thrust

        return u
    
    def vg_bounded(self):
        while not rospy.is_shutdown() and self.min_range_norm > 0.3:
            if self.tmp_center == self.center:
                self.balloon_in_frame = False
            # else:
            #     balloon_in_frame = True

            r,v = self.create_vg_state(self.balloon_in_frame) #get relativerange and velocity in local frame

            tgo = self.tgo_bounded(r, v, self.rho_u, self.rho_v, m=0.0, min_tgo=0.001) # calc tgo

            u = self.cont_bounded(tgo, r, v, max_thrust=0.7) # calc accelaration command fro VG bounded

            self.acc_cmd.vector.x = u[0]   #construct accelaration msg
            self.acc_cmd.vector.y = u[1]
            self.acc_cmd.vector.z = u[2] + 0.3

            self.acc_pub.publish(self.acc_cmd)  #publish accelaration msg

            self.tmp_center = self.center

            self.rate.sleep()

    def get_balloon_real_position(self):
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        balloon_coordinates = model_coordinates("green_sphere", "link")
        
        baloon_pose_x = balloon_coordinates.pose.position.x
        baloon_pose_y = balloon_coordinates.pose.position.y
        baloon_pose_z = balloon_coordinates.pose.position.z
        
        self.real_baloon_pose = np.array([baloon_pose_x, baloon_pose_y, baloon_pose_z])
        rospy.loginfo_once('Real balloon position = {}'.format(self.real_baloon_pose))

    def run(self):
        self.initialPose()
        while not rospy.is_shutdown() and self.min_range_norm > 0.3:
            try:
                self.get_balloon_real_position()
                self.counter = 0
                self.center = None
                self.scanning()
                self.vg_bounded()
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
