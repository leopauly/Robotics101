
#### Task 2

#### Imports
import sys
import rospy
import math
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_commander import PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Pose
import moveit_msgs.msg 
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from trajectory_msgs.msg import JointTrajectoryPoint
import geometry_msgs.msg
from tf import transformations
from copy import deepcopy
#from tf2_ros import Quaternion
import tf2_ros
import tf
import math

tau=2*math.pi

#### Initialise a robotics node
roscpp_initialize(sys.argv)
rospy.init_node('picknplace_node', anonymous=True)
print('Pick n Place Node initialised')

#### MoveIt APIs initialising
scene = PlanningSceneInterface()
robot = RobotCommander()
group_name="panda_arm"
move_group = MoveGroupCommander(group_name)


#### Details
group_names = robot.get_group_names()
print('Robot groups available:',group_names)
planning_frame = move_group.get_planning_frame()
print('Reference frame:',planning_frame)
eef_link = move_group.get_end_effector_link()
print('End effector link:',planning_frame)
print('Robot current state:',robot.get_current_state())
rospy.sleep(2)

#### Remove any pending objects
scene.remove_world_object("table1")
scene.remove_world_object("table2")
rospy.sleep(5) 

#### Adding new object
table2 = PoseStamped()
table2.header.frame_id = planning_frame
table2.pose.position.x = 0.0
table2.pose.position.y = -0.5
table2.pose.position.z = 0.2
scene.add_box("table2", table2, (0.4, 0.2, 0.4))
      	

#### Details
group_names = robot.get_group_names()
print('Robot groups available:',group_names)
planning_frame = move_group.get_planning_frame()
print('Reference frame:',planning_frame)
eef_link = move_group.get_end_effector_link()
print('End effector link:',planning_frame)
print('Robot current state:',robot.get_current_state())
rospy.sleep(2)

#### Placing pose definition
place_pose=PoseStamped()
place_pose.header.frame_id = planning_frame
quaternion = tf.transformations.quaternion_from_euler(0, 0, -tau / 4)
place_pose.pose.position.x = 0
place_pose.pose.position.y = -0.5
place_pose.pose.position.z = 0.5
place_pose.pose.orientation.x=quaternion[0]
place_pose.pose.orientation.y=quaternion[1]
place_pose.pose.orientation.z=quaternion[2]
place_pose.pose.orientation.w=quaternion[3]

#### Checking place pose 
#move_group.set_pose_target(place_pose)
#move_group.go()

place_location=PlaceLocation()
place_location.place_pose.header.frame_id = planning_frame
place_location.place_pose=place_pose


#### Pre-place approach
place_location.pre_place_approach.direction.header.frame_id =  planning_frame
place_location.pre_place_approach.direction.vector.z = -1.0
place_location.pre_place_approach.min_distance = 0.095
place_location.pre_place_approach.desired_distance = 0.115


#### Placing
move_group.set_support_surface_name("table2")
move_group.place("object1", place_location)


