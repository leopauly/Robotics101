
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

#### Publisher for publishing the trajectories 
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=20)

#### Adding objects into the scene
## Q: Add the following set of objects with as per the specifications given

table_id = 'table'
target_id = 'target'

table_ground = 0.65       
table_size = [0.2, 0.7, 0.1]   
target_size = [0.2, 0.1, 0.2]

table_pose = PoseStamped()
table_pose.header.frame_id = planning_frame
table_pose.pose.position.x = 0.55
table_pose.pose.position.y = 0.0
table_pose.pose.position.z = table_ground + table_size[2] / 2.0
table_pose.pose.orientation.w = 1.0
scene.add_box(table_id, table_pose, table_size)
        
target_pose = PoseStamped()
target_pose.header.frame_id = planning_frame
target_pose.pose.position.x = 0.55
target_pose.pose.position.y = -0.1
target_pose.pose.position.z = table_ground + table_size[2] + target_size[2] / 2.0
target_pose.pose.orientation.w = 1.0   
scene.add_box(target_id, target_pose, target_size)

#### Specifying and executing a goal in joint states
tau = 2*math.pi
joint_goal = move_group.get_current_joint_values()
joint_goal[0] = 0
joint_goal[1] = -tau / 8
joint_goal[2] = 0
joint_goal[3] = -tau / 4
joint_goal[4] = 0
joint_goal[5] = tau / 6  # 1/6 of a turn
joint_goal[6] = 0

move_group.go(joint_goal, wait=True)
move_group.stop()

#### Move the robotics arm towards a pre-defined end effector pose
## Q: Move the end-effector to the predefined locaiton with value (x,y,z)=[.4,.1,.4] & w=[1]
## Hint: A) Set pose_goal using an instace of geometry_msgs.msg.Pose() B) Use function move_group.set_pose_target(); C)Execute the pose goal using function move_group.go()
pose_goal = geometry_msgs.msg.Pose()
pose_goal.orientation.w = 1.0
pose_goal.position.x = 0.4
pose_goal.position.y = 0.1
pose_goal.position.z = 0.4

move_group.set_pose_target(pose_goal)
move_group.go(wait=True)

#### For stopping and clearing pose targets
move_group.stop()
move_group.clear_pose_targets()



        
#### Set the support surface name to the table object
move_group.set_support_surface_name(table_id)
        








