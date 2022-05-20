#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import numpy as np

class Balloon():
    def __init__(self):
        self.cmd_balloon = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.pose_balloon = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.cmd =  ModelState()
        self.rate = rospy.Rate(100)
    
    def est_range(self):
        balloon = self.pose_balloon('green_sphere', "")
        balloon_x = balloon.pose.position.x
        balloon_y = balloon.pose.position.y
        balloon_z = balloon.pose.position.z
        true_range = np.linalg.norm((self.pose.pose.position.x - balloon_x, self.pose.pose.position.y - balloon_y, self.pose.pose.position.z - balloon_z))
        print("Range = {}".format(true_range))
        return true_range

    def command_balloon(self, x, y, z):
        self.cmd.model_name = 'green_sphere'
        self.cmd.pose.position.x = x
        self.cmd.pose.position.y = y
        self.cmd.pose.position.z = z
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            self.cmd_balloon(self.cmd)
            pass
        except rospy.ServiceException as e:
            print(e)

def main():
    rospy.init_node('balloon_trajectory', anonymous=True, disable_signals=True)
    rospy.loginfo_once('***** Initiate Balloon Trajectory Node *****')
    balloon = Balloon()
    i = 0
    height = 5.0
    while not rospy.is_shutdown():
        radius = 10.0
        x = radius * np.sin(i)
        y = radius * np.cos(i)
        if i > 6.28:
            i = 0
        i += 0.01
        balloon.command_balloon(x, y, height)
        balloon.rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
        pass
