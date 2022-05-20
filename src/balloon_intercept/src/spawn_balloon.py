## spawn a balloon in the running gazebo simulaiton 
## user will be asked to enter the baloon coordinates in the terminal
## balloon gravity is off - will remain in the air


from turtle import position
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


rospy.init_node('insert_object',log_level=rospy.INFO)

initial_pose = Pose()
print("----ENTER BALLOON POSITION:----")
initial_pose.position.x = (input("x position: "))
initial_pose.position.y = (input("y position: "))
initial_pose.position.z = (input("z position: "))

f = open('/home/omri/.gazebo/models/green_sphere/model.sdf','r')
sdff = f.read()

rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
spawn_model_prox("green_sphere", sdff, "robotos_name_space", initial_pose, "world")

print("BALLOON SPAWNED AT:\n" + str(initial_pose.position))
