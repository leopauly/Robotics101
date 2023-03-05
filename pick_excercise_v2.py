
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

'''
#### Open and close gripper
move_group_gripper.set_named_target("close")
move_group_gripper.go()
move_group_gripper.set_named_target("open")
move_group_gripper.go()
rospy.sleep(5)
'''

#### Details
group_names = robot.get_group_names()
print('Robot groups available:',group_names)
planning_frame = move_group.get_planning_frame()
print('Reference frame:',planning_frame)
eef_link = move_group.get_end_effector_link()
print('End effector link:',planning_frame)
print('Robot current state:',robot.get_current_state())
rospy.sleep(2)

#### Publisher for publishing the trajectories 
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)
rospy.sleep(2)

#### Remove any pending objects
scene.remove_world_object("table")
scene.remove_world_object("box1")
scene.remove_world_object("box2")
scene.remove_world_object("target")
rospy.sleep(5)

#### Placing objects in scene
table_id = 'table'
box1_id = 'box1'
box2_id = 'box2'
target_id = 'target'
tool_id = 'tool'

table_ground = 0.65       
table_size = [0.2, 0.7, 0.01]
box1_size = [0.1, 0.05, 0.05]
box2_size = [0.05, 0.05, 0.15]       
target_size = [0.02, 0.01, 0.12]

def create_objects():
	table_pose = PoseStamped()
	table_pose.header.frame_id = planning_frame
	table_pose.pose.position.x = 0.55
	table_pose.pose.position.y = 0.0
	table_pose.pose.position.z = ( table_ground + table_size[2] / 2.0) -0.05
	table_pose.pose.orientation.w = 1.0
	scene.add_box(table_id, table_pose, table_size)
		
	box1_pose = PoseStamped()
	box1_pose.header.frame_id = planning_frame
	box1_pose.pose.position.x = 0.55
	box1_pose.pose.position.y = -0.3
	box1_pose.pose.position.z = table_ground + table_size[2] + box1_size[2] / 2.0
	box1_pose.pose.orientation.w = 1.0   
	scene.add_box(box1_id, box1_pose, box1_size)
		
	box2_pose = PoseStamped()
	box2_pose.header.frame_id = planning_frame
	box2_pose.pose.position.x = 0.54
	box2_pose.pose.position.y = 0.33
	box2_pose.pose.position.z = table_ground + table_size[2] + box2_size[2] / 2.0
	box2_pose.pose.orientation.w = 1.0   
	scene.add_box(box2_id, box2_pose, box2_size)       
		

	target_pose = PoseStamped()
	target_pose.header.frame_id = planning_frame
	target_pose.pose.position.x = 0.58
	target_pose.pose.position.y = 0.0
	target_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
	target_pose.pose.orientation.w = 1.0
	scene.add_box(target_id, target_pose, target_size)


create_objects()
rospy.sleep(5)     

#### Set the support surface name to the table object
move_group.set_support_surface_name(table_id)

#### Defining a grasp
grasps=[]
g = Grasp()
g.id = "test"
grasp_pose = PoseStamped()
grasp_pose.header.frame_id = planning_frame
grasp_pose.pose.position.x = .415
grasp_pose.pose.position.y = 0
grasp_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
tau=math.pi*2
quaternion = tf.transformations.quaternion_from_euler(-tau / 4, -tau / 8, -tau / 4)
grasp_pose.pose.orientation.x=quaternion[0]
grasp_pose.pose.orientation.y=quaternion[1]
grasp_pose.pose.orientation.z=quaternion[2]
grasp_pose.pose.orientation.w=quaternion[3]
g.grasp_pose = grasp_pose


#### To test grasp pose   
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
g.allowed_touch_objects = ["target_id"]

#### Grasping
grasps.append(g)
move_group.pick("target_id", grasps)





