
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

#### Initialise a robotics node
roscpp_initialize(sys.argv)
rospy.init_node('picknplace_node', anonymous=True)
print('Pick n Place Node initialised')

#### MoveIt APIs initialising
scene = PlanningSceneInterface()
robot = RobotCommander()
group_name="panda_arm"
move_group = MoveGroupCommander(group_name)
move_group_gripper = MoveGroupCommander("hand")

#### Open and close gripper
move_group_gripper.set_named_target("close")
move_group_gripper.go()
move_group_gripper.set_named_target("open")
move_group_gripper.go()
rospy.sleep(5)


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
scene.remove_world_object("object1")
rospy.sleep(5)

#### Placing objects
def create_objects():

	table1 = PoseStamped()
	table1.header.frame_id = planning_frame
	table1.pose.position.x = 0.5
	table1.pose.position.y = 0.0
	table1.pose.position.z = 0.2
	scene.add_box("table1", table1, (0.2, 0.4, 0.4))

	table2 = PoseStamped()
	table2.header.frame_id = planning_frame
	table2.pose.position.x = 0.0
	table2.pose.position.y = 0.5
	table2.pose.position.z = 0.2
	scene.add_box("table2", table2, (0.4, 0.2, 0.4))
        	
	object1 = PoseStamped()
	object1.header.frame_id = planning_frame
	object1.pose.position.x = 0.5
	object1.pose.position.y = 0.0
	object1.pose.position.z = 0.5
	scene.add_box("object1", object1, (0.02, 0.02, 0.2))

	move_group.set_support_surface_name("table1")


create_objects()
rospy.sleep(5)


#### Defining a grasp
grasps=[]
g = Grasp()
g.id = "pick_grasp"
grasp_pose = PoseStamped()
grasp_pose.header.frame_id = planning_frame
grasp_pose.pose.position.x = .415
grasp_pose.pose.position.y = 0.0
grasp_pose.pose.position.z = .5
tau=math.pi*2
quaternion = tf.transformations.quaternion_from_euler(-tau / 4, -tau / 8, -tau / 4)
grasp_pose.pose.orientation.x=quaternion[0]
grasp_pose.pose.orientation.y=quaternion[1]
grasp_pose.pose.orientation.z=quaternion[2]
grasp_pose.pose.orientation.w=quaternion[3]
g.grasp_pose = grasp_pose

#### Check pose grasp   
#move_group.set_pose_target(grasp_pose)
#move_group.go()


#### Pre-grasp approach
g.pre_grasp_approach.direction.header.frame_id = planning_frame
g.pre_grasp_approach.direction.vector.x = 1.0
g.pre_grasp_approach.min_distance = 0.095
g.pre_grasp_approach.desired_distance = 0.115


#### set the pre-grasp posture
g.pre_grasp_posture.header.frame_id = planning_frame
g.pre_grasp_posture.joint_names = ["panda_finger_joint1","panda_finger_joint2"]

pos_open = JointTrajectoryPoint()
pos_open.positions.append(float(0.04))
g.pre_grasp_posture.points.append(pos_open)
#g.pre_grasp_posture.points.append(pos_open)

#### set the post-grasp retreat
g.post_grasp_retreat.direction.header.frame_id = planning_frame
g.post_grasp_retreat.direction.vector.z = 1.0
g.post_grasp_retreat.min_distance = 0.1
g.post_grasp_retreat.desired_distance = 0.25


#### set the grasp posture
g.grasp_posture.header.frame_id = planning_frame
g.grasp_posture.joint_names = ["panda_finger_joint1","panda_finger_joint2"]

pos_close = JointTrajectoryPoint()
pos_close.positions.append(float(0.00))
g.grasp_posture.points.append(pos_close)
#g.grasp_posture.points.append(pos_close)

#### Other grasp settings
g.allowed_touch_objects = ["object1"]

#### Grasping
grasps.append(g)
move_group.pick("object1", grasps)

#### Placing
place_location=PlaceLocation()
place_location.place_pose.header.frame_id = planning_frame
quaternion = tf.transformations.quaternion_from_euler(0, 0, tau / 4)
place_location.place_pose.pose.position.x = 0
place_location.place_pose.pose.position.y = 0.5
place_location.place_pose.pose.position.z = 0.5
place_location.place_pose.pose.orientation.x=quaternion[0]
place_location.place_pose.pose.orientation.y=quaternion[1]
place_location.place_pose.pose.orientation.z=quaternion[2]
place_location.place_pose.pose.orientation.w=quaternion[3]

place_location.pre_place_approach.direction.header.frame_id =  planning_frame
place_location.pre_place_approach.direction.vector.z = -1.0
place_location.pre_place_approach.min_distance = 0.095
place_location.pre_place_approach.desired_distance = 0.115

place_location.post_place_retreat.direction.header.frame_id = planning_frame
place_location.post_place_retreat.direction.vector.y = -1.0;
place_location.post_place_retreat.min_distance = 0.1;
place_location.post_place_retreat.desired_distance = 0.25;


place_location.post_place_posture.header.frame_id = planning_frame
place_location.post_place_posture.joint_names = ["panda_finger_joint1","panda_finger_joint2"]
place_location.post_place_posture.points.append(pos_open)


move_group.set_support_surface_name("table2")
move_group.place("object1", place_location)


