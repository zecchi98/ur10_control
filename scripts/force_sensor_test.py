#!/usr/bin/env python
#fold all: ctrl + k + 0
#unfold all: ctrl + k + j
import copy
import math
from pickle import TRUE
from shutil import move
import sys
import time
from logging import setLoggerClass
from math import cos, pi, sin
from os import access
from re import X
import geometry_msgs.msg
import moveit_commander
import moveit_msgs.msg
import numpy as np
import rospy
from moveit_commander import *
from moveit_commander.conversions import pose_to_list
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander.robot import RobotCommander
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from ur10_control.srv import * 
from ur10_control.msg import *
#from noether_msgs.msg import *
from visualization_msgs.msg import *
#import open3d as o3d
from robodk import robolink, robomath      # import the robotics toolbox
from scipy.spatial.transform import Rotation 
import rospkg
from tf2_msgs.msg import TFMessage

bool_robodk_enabled=False
flagMiddlePanelCreated=False
bool_exit=False
bool_move_group_initialized=False
class Transformation_class():
  def __init__(self):
        null=0
  def rad_to_grad(self,angle):
    return angle*180/3.1415
  def grad_to_rad(self,angle):
    return angle*3.1415/180
  def rot2eul(self,R):
    beta = -np.arcsin(R[2,0])
    alpha = np.arctan2(R[2,1]/np.cos(beta),R[2,2]/np.cos(beta))
    gamma = np.arctan2(R[1,0]/np.cos(beta),R[0,0]/np.cos(beta))
    return np.array((alpha, beta, gamma))
  def rpy_from_quat (self,quaternion):
    orientation_list = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    return [roll,pitch,yaw]
  def Rotation_from_quat(self,quaternion):
    
    euler_angles=self.rpy_from_quat(quaternion)
    return self.eul2rot(euler_angles)
  def eul2rot(self,theta) :

    R = np.array([[np.cos(theta[1])*np.cos(theta[2]),       np.sin(theta[0])*np.sin(theta[1])*np.cos(theta[2]) - np.sin(theta[2])*np.cos(theta[0]),      np.sin(theta[1])*np.cos(theta[0])*np.cos(theta[2]) + np.sin(theta[0])*np.sin(theta[2])],
                  [np.sin(theta[2])*np.cos(theta[1]),       np.sin(theta[0])*np.sin(theta[1])*np.sin(theta[2]) + np.cos(theta[0])*np.cos(theta[2]),      np.sin(theta[1])*np.sin(theta[2])*np.cos(theta[0]) - np.sin(theta[0])*np.cos(theta[2])],
                  [-np.sin(theta[1]),                        np.sin(theta[0])*np.cos(theta[1]),                                                           np.cos(theta[0])*np.cos(theta[1])]])

    return R
  def Rotation_matrix_of_Affine_matrix(self,aff_matrix):
    R=np.zeros([3,3])
    for r in range(3):
      for c in range(3):
        R[r,c]=aff_matrix[r,c]
    return R
  def Translation_vector_of_Affine_matrix(self,AffineMat):
    vet=np.zeros(3)
    for r in range(3):
      vet[r]=AffineMat[r,3]
    return vet
  def create_affine_matrix_from_rotation_matrix_and_translation_vector(self,R,transl):
    #input: una matrice di rotazione e un vettore riga di traslazione"[0,1,3]"
    #return the result affine matrix
    AffineMat=np.zeros([4,4])
    
    #copio matrice di rotazione
    for r in range(3):
      for c in range(3):
        AffineMat[r,c]=R[r,c]

    #copio vettore traslazione
    AffineMat[0,3]=transl[0]
    AffineMat[1,3]=transl[1]
    AffineMat[2,3]=transl[2]

    #setto ultima riga standard
    AffineMat[3,0]=0
    AffineMat[3,1]=0
    AffineMat[3,2]=0
    AffineMat[3,3]=1

    return AffineMat
  def from_euler_to_quaternion(self,euler_vet):
    #Input: vettore degli angoli di eulero
    #Output: Pose with the correct orientation in quaternion
    roll=euler_vet[0]
    pitch=euler_vet[1]
    yaw=euler_vet[2]
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    pose=Pose()
    pose.orientation.x=qx
    pose.orientation.y=qy
    pose.orientation.z=qz
    pose.orientation.w=qw
    
    return pose 
  def from_rotation_to_quaternion(self,R):
    #Input: Rotation matrix
    #Output: Pose with the correct orientation in quaternion
    euler_vet=self.rot2eul(R)
    pose_oriented=self.from_euler_to_quaternion(euler_vet)
    return pose_oriented
  def from_vet_to_posePosition(self,vet):
    #Input: vettore contentente la posizione di un frame
    #Output: Pose con la corretta position
    pose=Pose()
    pose.position.x=vet[0]
    pose.position.y=vet[1]
    pose.position.z=vet[2]
    return pose
  def from_pose_to_matrix(self,pose_):
    R=self.Rotation_from_quat(pose_.orientation)
    vet=[pose_.position.x,pose_.position.y,pose_.position.z]
    AffineMat=self.create_affine_matrix_from_rotation_matrix_and_translation_vector(R,vet)
    return AffineMat
  def from_matrix_to_pose(self,AffineMat):
    #Input: Affine matrix
    #Output: Pose
    pose=Pose()
    
    R=self.Rotation_matrix_of_Affine_matrix(AffineMat)
    translation_vet=self.Translation_vector_of_Affine_matrix(AffineMat)
    
    #pose with the correct position
    pose_position=self.from_vet_to_posePosition(translation_vet)

    #pose with the correct orientation
    pose_orientation=self.from_rotation_to_quaternion(R)

    pose.orientation=pose_orientation.orientation
    pose.position=pose_position.position
    return pose
  def inverse_matrix(self,AffineMat):
    return np.linalg.inv(AffineMat)
  def compute_distance_pose(self,pose1,pose2):
    x1=pose1.position.x
    y1=pose1.position.y
    z1=pose1.position.z
    x2=pose2.position.x
    y2=pose2.position.y
    z2=pose2.position.z
    dx=(x1-x2)*(x1-x2)
    dy=(y1-y2)*(y1-y2)
    dz=(z1-z2)*(z1-z2)
    return math.sqrt(dx+dy+dz)

class Collision_Box():
    def __init__(self):
        self.box_pose = PoseStamped()
        self.box_size = np.zeros(3)
        self.box_name = String()  
class Affine_valid():
    def __init__(self):
        self.is_valid = False
        self.Affine_matrix = np.zeros((4,4))
class Pose_valid():
    def __init__(self):
        self.is_valid = False
        self.pose = Pose()
class Move_group_class(object):
  """Move_group_class"""
  def __init__(self):
    super(Move_group_class, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('state_machine', anonymous=True)

    ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    ## kinematic model and the robot's current joint states
    
    robot = RobotCommander()
    
    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = PlanningSceneInterface()
    
    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  In this tutorial the group is the primary
    ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
    ## If you are using a different robot, change this value to the name of your robot
    ## arm planning group.
    ## This interface can be used to plan and execute motions:
    group_name = "manipulator"
    
    move_group =MoveGroupCommander(group_name)
    
    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## Getting Basic Information
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    
    print ("============ Planning frame: %s" + planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print ("============ End effector link: %s" + eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
   # print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    #print "============ Printing robot state"
    #print robot.get_current_state()
    #print ""
    ## END_SUB_TUTORIAL
    
    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names
  def all_close(self,goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
      for index in range(len(goal)):
        if abs(actual[index] - goal[index]) > tolerance:
          return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
      return self.all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
      return self.all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True
  def follow_pose_trajectory(self,pose_array):
    nul=0
    move_group = self.move_group

    
    (plan, fraction) = move_group.compute_cartesian_path(
                                       pose_array,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    
    move_group.execute(plan, wait=True)
  def ruota_giunto(self,id_giunto,angle):
    joints=self.get_joints_values()
    joints[id_giunto]=joints[id_giunto]+angle
    
    self.go_to_joint_state(joints)
  def go_to_joint_state(self,joints_vet):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## Planning to a Joint Goal
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
    ## thing we want to do is move it to a slightly better configuration.
    # We can get the joint values from the group and adjust some of the values:
    

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group

    for i in range(0,len(joints_vet)):
      if(joints_vet[i]>3.14 or joints_vet[i]<-3.14):
        print("Out of bounds")
        return

    move_group.go(joints_vet, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return self.all_close(joints_vet, current_joints, 0.01)
  def go_to_pose_goal(self,pose_goal,RandomOrientation=False):
    move_group = self.move_group
    
    if not RandomOrientation:
      move_group.set_pose_target(pose_goal)
    else:
      pos=Point()
      xyz=[pose_goal.position.x,pose_goal.position.y,pose_goal.position.z]
      move_group.set_position_target(xyz)
    
    plan = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    current_pose = self.move_group.get_current_pose().pose
    return self.all_close(pose_goal, current_pose, 0.01)
  def go_to_pose_cartesian(self,pose_goal,bool_allow_std_movement=False):
  
    move_group = self.move_group

    waypoints = []

    waypoints.append(copy.deepcopy(pose_goal))

    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold
    
    #Se fraction <1 e quindi c'e stato un errore, allora utilizza il planner normale
    if(bool_allow_std_movement and fraction<1):
      print(fraction)
      self.go_to_pose_goal(pose_goal,False)
      #Esco senza return
      return
    
    self.execute_plan(plan)
    self.display_trajectory(plan)

    return plan, fraction
  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## Cartesian Paths
    ## ^^^^^^^^^^^^^^^
    ## You can plan a Cartesian path directly by specifying a list of waypoints
    ## for the end-effector to go through. If executing  interactively in a
    ## Python shell, set scale = 1.0.
    ##
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction
  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)
  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)
  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
  def add_box(self,collision_box, timeout=0):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    #box_name = self.box_name
    #print('adding box')
    #print(box_name)
    #print(box_size)
    #print(box_pose)
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    #box_pose = geometry_msgs.msg.PoseStamped()
    #box_pose.header.frame_id = "base_link"
    #box_pose.pose.orientation.w = 1.0
    #box_pose.pose.position.x = -0.1 # slightly above the end effector
    #box_name = "box"
    size=(collision_box.box_size[0],collision_box.box_size[1],collision_box.box_size[2])
    scene.add_box(collision_box.box_name, collision_box.box_pose, size)
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)
  def attach_box(self,box_pose,box_name,box_size, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    grasping_group = 'manipulator'
    #touch_links = robot.get_link_names()
    touch_links=['wrist_3_link', 'ee_link', 'tool0', 'camera_ur_mount', 'camera_link1', 'camera_link', 'camera_camera_lens', 'camera_camera', 'camera_camera_gazebo', 'robotiq_arg2f_base_link', 'left_outer_knuckle', 'left_outer_finger', 'left_inner_finger', 'left_finger_tip_temp', 'left_finger_tip', 'left_inner_finger2', 'left_inner_knuckle', 'left_inner_knuckle2', 'plate1', 'dys_middle', 'right_inner_knuckle', 'right_inner_knuckle2', 'right_outer_knuckle', 'right_outer_finger', 'right_inner_finger', 'right_finger_tip_temp', 'right_finger_tip', 'right_inner_finger2']

    #print(touch_links)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)
  def detach_box(self,box_pose,box_name,box_size, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    scene = self.scene
    eef_link = self.eef_link

    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)
  def remove_box(self,box_name,timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    #box_name = self.box_name
    scene = self.scene
    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)
    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)
  def get_joints_values(self):
    joints_grad=movegroup_library.move_group.get_current_joint_values()
    # joints_rad=np.zeros(6)
    # for i in range(6):
    #   joints_rad[i]=transformation_library.grad_to_rad(joints_grad[i])
    return joints_grad
  def FermaRobot(self):
    print("Il robot sta per essere fermato")
    self.move_group.stop()
    self.move_group.clear_pose_targets()
  def Stampa_Info_Robot(self):
    print("\n\nRobot Status")
    pose=self.move_group.get_current_pose().pose
    print("Pose:")
    print(pose)

    joints=self.get_joints_values()
    print("\nJoints:")
    print(joints)
    
    rpy=transformation_library.rpy_from_quat(pose.orientation)
    print("\nRpy:")
    print(rpy)
    print("\n\nActive joints:")
    print(self.move_group.get_active_joints())
    aruco_library.print_situation_aruco()
  def Stampa_rpy(self):
    
    print("\n\nRobot RPY")
    pose=self.move_group.get_current_pose().pose
    rpy=transformation_library.rpy_from_quat(pose.orientation)
    print("\nRpy:")
    print(rpy)
  def get_current_ee_pose(self):
    return self.move_group.get_current_pose()
  def get_T_base_camera_depth(self):
    trans=[0,0,0]
    R=transformation_library.eul2rot([0,0,math.pi])
    T_sr_change=transformation_library.create_affine_matrix_from_rotation_matrix_and_translation_vector(R,trans)

    R_tool_camera=transformation_library.eul2rot([0,-math.pi/2,math.pi/2])
    trans_tool_camera=[0,0,0]
    T_tool_camera=transformation_library.create_affine_matrix_from_rotation_matrix_and_translation_vector(R_tool_camera,trans_tool_camera)

    R_camera_camera_depth=transformation_library.eul2rot([-math.pi/2,0,-math.pi/2])
    trans_camera_camera_depth=[0,0,0]
    T_camera_camera_depth=transformation_library.create_affine_matrix_from_rotation_matrix_and_translation_vector(R_camera_camera_depth,trans_camera_camera_depth)


    R_ee_tool=transformation_library.eul2rot([-math.pi/2,0,-math.pi/2])
    trans_ee_tool=[0,0,0]
    T_ee_tool=transformation_library.create_affine_matrix_from_rotation_matrix_and_translation_vector(R_ee_tool,trans_ee_tool)


    pose_ee=movegroup_library.get_current_ee_pose().pose
    T_base_ee=transformation_library.from_pose_to_matrix(pose_ee)

    #T_base_ee_changed=np.dot(T_sr_change,T_base_ee)
    T_base_tool=np.dot(T_base_ee,T_ee_tool)
    T_tool_camera_depth=np.dot(T_tool_camera,T_camera_camera_depth)
    T_base_camera_depth=np.dot(T_base_tool,T_tool_camera_depth)
    return T_base_camera_depth
class Comunication_class(object):
  def __init__(self):
    super(Comunication_class, self).__init__()
    self.bridge_service_information = rospy.ServiceProxy('cv_server', cv_server)
  def call_cv_service(self,first_information,second_information):
    
    try:
          req=cv_serverRequest()
          req.message=first_information
          req.second_information=second_information

          msg = self.bridge_service_information(req)

    except rospy.ServiceException as e:
      print("Service call failed: %s"%e)
  def print_bridge_response(self,response):
    
    print("Aruco trovato:"+str(response.aruco_found))
    print("ID Aruco:"+str(response.id_aruco))
  def ask_matrix_camera_aruco(self):
    ValidMatrix=Affine_valid()
    msg=self.read_data_from_cv()
    if not msg.aruco_found:
      ValidMatrix.is_valid=False
      return ValidMatrix
    ValidMatrix.is_valid=True
    translation_vet=[msg.x,msg.y,msg.z]
    R=np.matrix([[msg.vector[0],msg.vector[1],msg.vector[2]],[msg.vector[3],msg.vector[4],msg.vector[5]],[msg.vector[6],msg.vector[7],msg.vector[8]]])
    
    AffineMat=transformation_library.create_affine_matrix_from_rotation_matrix_and_translation_vector(R,translation_vet)
    ValidMatrix.Affine_matrix=AffineMat
    return ValidMatrix
  def read_data_from_cv(self):
    try:
      msg=rospy.wait_for_message("/aruco_bridge_opencv",cv_to_bridge)
      resp=cv_to_bridge()
      resp.aruco_found=msg.aruco_found
      resp.pose_camera_aruco=msg.pose_camera_aruco
      return resp
    except:
      print("error reading from cv")
  def matrix_from_cv_of_specified_aruco(self,id_aruco):
    data=self.read_data_from_cv()
    ValidMatrix=Affine_valid()
    if(data.aruco_found[int(id_aruco)]):
      ValidMatrix.is_valid=True
      ValidMatrix.Affine_matrix=transformation_library.from_pose_to_matrix(data.pose_camera_aruco[int(id_aruco)])
    else:
      ValidMatrix.is_valid=False
    return ValidMatrix
class Noether_comunication(object):
  def __init__(self):
    global marker_cont
    super(Noether_comunication, self).__init__()
    #rospy.Subscriber("/generate_tool_paths/result",GenerateToolPathsActionResult,self.noether_result_callback)
    rospack = rospkg.RosPack()
    pathTopkg=rospack.get_path('pcl_zk')
    pathToFile=pathTopkg+"/data/traiettoria.txt"
    self.all_poses_in_traj=[]
    self.all_normals=[]
    self.real_traj=[]
    self.pathToFile=pathToFile
    self.bool_traj_saved=False
    self.bool_normals_saved=False
    self.bool_associazione_completata=False

    marker_cont=0
    self.cont_collision=0
  def noether_result_callback(self,msg):
    self.all_poses_in_traj=[]
    tool_paths=msg.result.tool_paths
    for tool_path in tool_paths:
      paths=tool_path.paths
      for path in paths:
        segments=path.segments
        for segment in segments:
          poses=segment.poses
          #movegroup_library.follow_pose_trajectory(poses)
          for pose in poses:
            #print(pose)
            #movegroup_library.go_to_pose_cartesian(pose)
            #time.sleep(1)
            add_pose_to_marker_array(pose)
            self.all_poses_in_traj.append(pose)
    self.save_traiettoria()
  def save_traiettoria(self):
    
    output_file = open(self.pathToFile, "w")
    for pose in self.all_poses_in_traj:
      x=str(pose.position.x)
      y=str(pose.position.y)
      z=str(pose.position.z)
      ox=str(pose.orientation.x)
      oy=str(pose.orientation.y)
      oz=str(pose.orientation.z)
      ow=str(pose.orientation.w)
      output_file.write(x+" "+y+" "+z+" "+ox+" "+oy+" "+oz+" "+ow+"\n")
    output_file.close()
    print("Finish to plot and save trajectory\n\n")
  def read_traj_from_file(self):
    nul=0
    self.all_poses_in_traj=[]
    input_file = open(self.pathToFile, "r")
    for x in input_file:
      s=x.split()
      pose=Pose()
      pose.position.x=float(s[0])
      pose.position.y=float(s[1])
      pose.position.z=float(s[2])
      pose.orientation.x=float(s[3])
      pose.orientation.y=float(s[4])
      pose.orientation.z=float(s[5])
      pose.orientation.w=float(s[6])
      self.all_poses_in_traj.append(pose)
    print("Finished to read traj from file")
    #print(self.all_poses_in_traj)
    self.bool_traj_saved=True
  def read_normals_from_file(self):
    rospack = rospkg.RosPack()
    pathTopkg=rospack.get_path('pcl_zk')
    pathTopkg=pathTopkg+"/data/"
    pathToFile=pathTopkg+"my_normals.pcd"
    input_file = open(pathToFile, "r")
    cont=0
    cont2=0
    for x in input_file:
      cont=cont+1
    
      if(cont>11):
        
        s=x.split()
        if(s[0]!="nan"):
          pos=movegroup_library.get_current_ee_pose().pose
          pos.position.x=float(s[0])
          pos.position.y=float(s[1])
          pos.position.z=float(s[2])
          quat=transformation_library.from_euler_to_quaternion([float(s[3]),float(s[4]),float(s[5])])
          pos.orientation=quat.orientation
          trans=[0,0,0]
          R=transformation_library.eul2rot([0,math.pi/2,0])
          T_normal=transformation_library.create_affine_matrix_from_rotation_matrix_and_translation_vector(R,trans)



          T_actual=transformation_library.from_pose_to_matrix(pos)

          T_final=np.dot(T_actual,T_normal)
          Pos_final=transformation_library.from_matrix_to_pose(T_final)


          self.all_normals.append(Pos_final)
    self.bool_normals_saved=True
    print("Normals reading completed")
  def associate_traj_and_normals_points(self):
    if not self.bool_traj_saved :
      self.read_traj_from_file()
    if not self.bool_normals_saved :
      self.read_normals_from_file()
     
    for traj_pose in self.all_poses_in_traj:
      min=1000
      for normal_pose in self.all_normals:

        dist=transformation_library.compute_distance_pose(traj_pose,normal_pose)
        if(dist<min):
          min=dist
          normal_piu_vicina=normal_pose
      
      self.real_traj.append(normal_piu_vicina)
    
    print("Associazione finita")
  def add_pose_to_marker_array_as_points(self,pose,sdr):
    global marker_cont,markerArray
    marker=Marker()
    #marker.header.frame_id = "cameradepth_link"
    marker.header.frame_id = sdr
    marker.header.stamp = rospy.get_rostime()

    marker.ns = ""
    marker.id = marker_cont
    marker.type = visualization_msgs.msg.Marker.ARROW
    marker.action = 0
    marker.pose=pose  
    
    #R1=transformation_library.Rotation_from_quat(marker.pose.orientation)
    #R2=transformation_library.eul2rot([0,math.pi/2,0])
    #R=np.dot(R1,R2)
    #pose_quat=transformation_library.from_rotation_to_quaternion(R)
    #marker.pose.orientation=pose_quat.orientation

    
    marker.scale.x = 0.01
    marker.scale.y = 0.002
    marker.scale.z = 0.002
    marker.color.a = 1.0; 
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 1.0


    markerArray.markers.append(marker)
    marker_cont=marker_cont+1        
  def add_pose_to_marker_array_as_arrows(self,pose,sdr):
    global marker_cont,markerArray
    marker=Marker()
    #marker.header.frame_id = "cameradepth_link"
    marker.header.frame_id = sdr
    marker.header.stamp = rospy.get_rostime()

    marker.ns = ""
    marker.id = marker_cont
    marker.type = visualization_msgs.msg.Marker.ARROW
    marker.action = 0
    marker.pose=pose  
    
    #R1=transformation_library.Rotation_from_quat(marker.pose.orientation)
    #R2=transformation_library.eul2rot([0,math.pi/2,0])
    #R=np.dot(R1,R2)
    #pose_quat=transformation_library.from_rotation_to_quaternion(R)
    #marker.pose.orientation=pose_quat.orientation

    marker.scale.x = 0.02
    marker.scale.y = 0.005
    marker.scale.z = 0.005
    marker.color.a = 1.0; 
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0


    markerArray.markers.append(marker)
    marker_cont=marker_cont+1        
  def publish_traj(self):
    markerArray=MarkerArray()
    for x in self.all_poses_in_traj:
      self.add_pose_to_marker_array_as_arrows(x,"base")

    pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    pub.publish(markerArray)
  def publish_traj_CAMERA(self):
    markerArray=MarkerArray()
    #print(len(self.all_poses_in_traj))
    for x in self.all_poses_in_traj:
      self.add_pose_to_marker_array_as_arrows(x,"cameradepth_link")

    pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    pub.publish(markerArray)
  def publish_traj_with_normals(self):
    markerArray=MarkerArray()
    #print(len(self.all_poses_in_traj))
    for x in self.real_traj:
      self.add_pose_to_marker_array_as_arrows(x,"base_link")

    pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    pub.publish(markerArray)
  def make_pointcloud_as_collision(self):
      print("rendo la poincloud collisibile")
      jump=0
      for x in self.all_normals:
        jump=jump+1
        if(jump%1==0):
          collision_box=Collision_Box()
          collision_box.box_name=str(self.cont_collision)
          collision_box.box_size[0]=0.001
          collision_box.box_size[1]=0.001
          collision_box.box_size[2]=0.001
          collision_box.box_pose.header.frame_id="cameradepth_link"
          pose_orientation=transformation_library.from_euler_to_quaternion([0,0,0])
          collision_box.box_pose.pose.orientation=pose_orientation.orientation
          collision_box.box_pose.pose.position.x=x.position.x
          collision_box.box_pose.pose.position.y=x.position.y
          collision_box.box_pose.pose.position.z=x.position.z
          movegroup_library.add_box(collision_box)
          self.cont_collision=self.cont_collision+1
  
  def CAMERA_leggi_normals(self):
    #this function works with all the points in the camera reference system
    rospack = rospkg.RosPack()
    pathTopkg=rospack.get_path('pcl_zk')
    pathTopkg=pathTopkg+"/data/"
    pathToFile=pathTopkg+"real_surface_points.pcd"
    
    input_file = open(pathToFile, "r")
    cont=0
    cont2=0
    self.all_normals=[]
    trans=[0,0,0]
    cont2=0
    for x in input_file:
            s=x.split()
            cont2=cont2+1

            pos=Pose()
            pos.position.x=float(s[0])
            pos.position.y=float(s[1])
            pos.position.z=float(s[2])
            pos.orientation.x=float(s[3])
            pos.orientation.y=float(s[4])
            pos.orientation.z=float(s[5])
            pos.orientation.w=float(s[6])
          
            #if(cont2%10000==0):
              #print(cont2)
            self.all_normals.append(pos)  
    
    self.bool_normals_saved=True   
    print("Finish to read normals")
  def CAMERA_associate_traj_and_normals_points(self):
    if self.bool_associazione_completata:
      return
    if not self.bool_traj_saved :
      self.read_traj_from_file()
    if not self.bool_normals_saved :
      self.CAMERA_leggi_normals()
    T_base_camera_depth=movegroup_library.get_T_base_camera_depth()
    print("Inizio ad eseguire l'associazione, ci vorra un po")
    print("Ricordati che ogni punto e' stato traslato di 1 cm per non collidere")
    trans=[-0.01,0,0]
    R=transformation_library.eul2rot([0,0,0])
    T_target_to_approach=transformation_library.create_affine_matrix_from_rotation_matrix_and_translation_vector(R,trans)

    for traj_pose in self.all_poses_in_traj:
      min=1000
      for normal_pose in self.all_normals:

        dist=transformation_library.compute_distance_pose(traj_pose,normal_pose)
        if(dist<min):
          min=dist
          normal_piu_vicina=normal_pose
      
      #Trasformo da camera_depth a base sdr
      pos_target=normal_piu_vicina
      T_from_camera_depth_TO_target=transformation_library.from_pose_to_matrix(pos_target)
      T_base_target=np.dot(T_base_camera_depth,T_from_camera_depth_TO_target)

      T_base_target_approach=np.dot(T_base_target,T_target_to_approach)

      final_target=transformation_library.from_matrix_to_pose(T_base_target_approach)
      #print(final_target)
      #print(pos_target)
      #time.sleep(1)

      self.real_traj.append(final_target)
    self.bool_associazione_completata=True
    print("Associazione finita")
  def go_to_all_normals_CAMERA(self):
    global bool_exit
    pose_array=self.all_normals
    #print(pose_array)
    T_base_camera_depth=movegroup_library.get_T_base_camera_depth()
    cont=0
    for pose in pose_array:
      #print(pose)
      cont=cont+1
      if cont%1000==0:
        #Questa rotazione ha lo scopo di mettere il tool sopra l'oggetto, normale ad esso
        T_from_camera_depth_TO_target=transformation_library.from_pose_to_matrix(pose)
        T_base_target=np.dot(T_base_camera_depth,T_from_camera_depth_TO_target)
        
        trans=[0,0,0]
        R=transformation_library.eul2rot([0,0,math.pi])
        T_sr_change=transformation_library.create_affine_matrix_from_rotation_matrix_and_translation_vector(R,trans)

        
        T_final=np.dot(T_sr_change,T_base_target)
        Pos_final=transformation_library.from_matrix_to_pose(T_base_target)
        movegroup_library.go_to_pose_goal(Pos_final)
        bool_exit=rospy.get_param("/CloseSystem")
        if bool_exit:
          return
  def publish_normals(self):
    global markerArray
    markerArray=MarkerArray()
    cont=0
    T_base_camera_depth=movegroup_library.get_T_base_camera_depth()
    for x in self.all_normals:
      cont=cont+1
      if cont%1000==0:
        self.add_pose_to_marker_array_as_points(x,"cameradepth_link")

    pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=10)
    pub.publish(markerArray)
def define_all_initial_functions():
  global comunication_object,transformation_library,movimenti_base_library,aruco_library
  global joystick_verso_rotazione,joystick_angle_step,joystick_translation_step,bool_message_from_user,markerArray,pub,marker_cont
  global xyz_actual,noether_library,movegroup_library,bool_move_group_initialized
  bool_message_from_user=False
  transformation_library=Transformation_class()
  noether_library=Noether_comunication()
  rospy.Subscriber("/tf",TFMessage,tf_subscriber)
  rospy.set_param("/CloseSystem",False)
  xyz_actual=[0,0,0]

  bool_move_group_initialized=True
  movegroup_library=Move_group_class()
  define_std_matrices()
def define_std_matrices():
  global T_base_camera_depth
  nul=0
  #da ee a tool -90 su x 90 su y

def publish_pointcloud_markers():

  rospack = rospkg.RosPack()
  pathTopkg=rospack.get_path('pcl_zk')
  pathTopkg=pathTopkg+"/data/"
  pathToFile=pathTopkg+"my_cloud.xyz"
  input_file=open(pathToFile, "r")
  for x in input_file:
    x=x.split()
    add_vector_to_marker_array(x)
  print(markerArray.markers)
def publish_percorso_calcolato():

  rospack = rospkg.RosPack()
  pathTopkg=rospack.get_path('ur10_control')
  pathTopkg=pathTopkg+"/Prova/"
  pathToFile=pathTopkg+"percorso.xyz"
  input_file=open(pathToFile, "r")
  for x in input_file:
    x=x.split()
    add_vector_to_marker_array(x)
    time.sleep(1)
    pub.publish(markerArray)
  print(markerArray.markers)
def add_vector_to_marker_array(vet):
  global marker_cont
  marker=Marker()
  marker.header.frame_id = "base"
  marker.header.stamp = rospy.get_rostime()

  marker.ns = ""
  marker.id = marker_cont
  marker.type = visualization_msgs.msg.Marker.SPHERE
  marker.action = 0
  marker.pose=Pose()
  marker.pose.position.x=float(vet[0])
  marker.pose.position.y=float(vet[1])
  marker.pose.position.z=float(vet[2])
  marker.pose.orientation.w=1
  
  #R1=transformation_library.Rotation_from_quat(marker.pose.orientation)
  #R2=transformation_library.eul2rot([0,math.pi/2,0])
  #R=np.dot(R1,R2)
  #pose_quat=transformation_library.from_rotation_to_quaternion(R)
  #marker.pose.orientation=pose_quat.orientation

  marker.scale.x = 0.01
  marker.scale.y = 0.01
  marker.scale.z = 0.01
  marker.color.a = 1.0; 
  marker.color.r = 0.0
  marker.color.g = 1.0
  marker.color.b = 0.0


  markerArray.markers.append(marker)
  marker_cont=marker_cont+1
def elaboraManualCloud():
  rospy.set_param("/elaborate_manual_pointcloud",True)
def salva_punto_nella_pointcloud():
  pose=movegroup_library.get_current_ee_pose()

  rospack = rospkg.RosPack()
  pathTopkg=rospack.get_path('pcl_zk')
  pathTopkg=pathTopkg+"/data/"
  pathToFile=pathTopkg+"my_cloud.xyz"
  
  output = open(pathToFile, "a")
  #x=pose.pose.position.x
  #y=pose.pose.position.y
  #z=pose.pose.position.z
  x=xyz_actual[0]
  y=xyz_actual[1]
  z=xyz_actual[2]
  output.write(str(x)+" "+str(y)+" "+str(z)+"\n")
def eliminare_ultimo_punto_cloud():

  value = input("\n\n Sto per eliminare un punto dalla cloud, sei sicuro?\n 1)Si\n 2)No\n Risposta:")
  if(value==2):
    return
  rospack = rospkg.RosPack()
  pathTopkg=rospack.get_path('pcl_zk')
  pathTopkg=pathTopkg+"/data/"
  pathToFile=pathTopkg+"my_cloud.xyz"
  input_file = open(pathToFile, "r")
  file_strings=input_file.readlines()
  file_strings.pop()
  input_file.close()

  output = open(pathToFile, "w")
  output.writelines(file_strings)
def go_to_all_poses_of_cloud():
  rospack = rospkg.RosPack()
  pathTopkg=rospack.get_path('pcl_zk')
  pathTopkg=pathTopkg+"/data/"
  pathToFile=pathTopkg+"my_normals.pcd"
  input_file = open(pathToFile, "r")
  cont=0
  cont2=0
  for x in input_file:
    cont=cont+1
    
    if(cont>11):
      s=x.split()
      if(s[0]!="nan"):
        print(s)
        cont2=cont2+1
        if(cont2>20):
          print("force to exit")
          return
        pos=movegroup_library.get_current_ee_pose().pose
        pos.position.x=float(s[0])
        pos.position.y=float(s[1])
        pos.position.z=float(s[2])
        #pos.position.x=0.1
        #pos.position.y=0
        #pos.position.z=0
        
        #pos.orientation.x=float(s[3])
        #pos.orientation.y=float(s[4])
        #pos.orientation.z=float(s[5])
        #pos.orientation.w=float(s[6])
        
        pos.position.z=pos.position.z+0
        quat=transformation_library.from_euler_to_quaternion([float(s[3]),float(s[4]),float(s[5])])
        pos.orientation=quat.orientation

        #Questa rotazione ha lo scopo di mettere il tool sopra l'oggetto, normale ad esso
        trans=[0,0,0]
        R=transformation_library.eul2rot([0,math.pi/2,0])
        T_normal=transformation_library.create_affine_matrix_from_rotation_matrix_and_translation_vector(R,trans)



        trans=[0,0,0]
        R=transformation_library.eul2rot([0,0,math.pi])
        T_sr_change=transformation_library.create_affine_matrix_from_rotation_matrix_and_translation_vector(R,trans)

        T_actual=transformation_library.from_pose_to_matrix(pos)


        T_final=np.dot(T_sr_change,T_actual)
        T_final=np.dot(T_final,T_normal)
        Pos_final=transformation_library.from_matrix_to_pose(T_final)
        print(Pos_final)
        movegroup_library.go_to_pose_goal(Pos_final)
def read_traj_from_file_and_publish():
  noether_library.read_traj_from_file()
  noether_library.publish_traj()
def go_to_all_poses_of_traj_file():
  global bool_exit
  
  noether_library.associate_traj_and_normals_points()
  pose_array=noether_library.real_traj
  for pose in pose_array:
    #transformation_library.from_euler_to_quaternion([0,0,0])

    #rotate sdr
    trans=[0,0,0]
    R=transformation_library.eul2rot([0,0,math.pi])
    T_sr_change=transformation_library.create_affine_matrix_from_rotation_matrix_and_translation_vector(R,trans)
    T_target=transformation_library.from_pose_to_matrix(pose)


    T_final=np.dot(T_sr_change,T_target)
    Pos_final=transformation_library.from_matrix_to_pose(T_final)
    movegroup_library.go_to_pose_goal(Pos_final)
    bool_exit=rospy.get_param("/CloseSystem")
    if bool_exit:
      return
def cambia_valore_di_alpha():
  alpha = input("\n Inserisci nuovo valore di alpha\n Risposta:")
  rospy.set_param("/alpha",alpha)


#pointcloud_from_camera
def salva_nuove_coordinate():
  rospack = rospkg.RosPack()
  pathTopkg=rospack.get_path('pcl_zk')
  pathTopkg=pathTopkg+"/data/"
  pathToFile=pathTopkg+"mesh.pcd"
  pathToNewCoordinates=pathTopkg+"new_coordinates.pcd"
  input_file = open(pathToFile, "r")
  output_file = open(pathToNewCoordinates, "w")
  cont=0
  cont2=0

  T_base_camera_depth=movegroup_library.get_T_base_camera_depth()
  pos=movegroup_library.get_current_ee_pose().pose
  for x in input_file:
    cont=cont+1
    


    if(cont>12):
      s=x.split()
      if(s[0]!="nan"):
        cont2=cont2+1
        #print(s)
        pos.position.x=float(s[0])
        pos.position.y=float(s[1])
        pos.position.z=float(s[2])
        #pos.position.x=0.1
        #pos.position.y=0
        #pos.position.z=0
        
        target_rpy_vet=[float(s[3]),float(s[4]),float(s[5])]
        quaternion_target=transformation_library.from_euler_to_quaternion(target_rpy_vet)
        pos.orientation=quaternion_target.orientation

        T_from_camera_depth_TO_target=transformation_library.from_pose_to_matrix(pos)
        

        T_base_target=np.dot(T_base_camera_depth,T_from_camera_depth_TO_target)
        
        target=transformation_library.from_matrix_to_pose(T_base_target)
        #target.orientation=movegroup_library.get_current_ee_pose().pose.orientation
        #print(target)
        #print(transformation_library.rpy_from_quat(target.orientation))
        output_file.writelines(str(target.position.x)+" "+str(target.position.y)+" "+str(target.position.z)+" "+str(target.orientation.x)+" "+str(target.orientation.y)+" "+str(target.orientation.z)+" "+str(target.orientation.w)+"\n")
def go_to_all_traj_pose_wrt_camera():
  global bool_exit
  #this function works with all the points in the camera reference system
  noether_library.CAMERA_associate_traj_and_normals_points()
  #noether_library.go_to_all_normals_CAMERA()
  #return
  pose_array=noether_library.real_traj
  #print(pose_array)
  for pose in pose_array:
    #print(pose)
    #Questa rotazione ha lo scopo di mettere il tool sopra l'oggetto, normale ad esso
    #trans=[0,0,0]
    #R=transformation_library.eul2rot([0,-math.pi/2,0])
    #T_normal=transformation_library.create_affine_matrix_from_rotation_matrix_and_translation_vector(R,trans)



    #T_actual=transformation_library.from_pose_to_matrix(pose)


    #T_final=np.dot(T_actual,T_normal)
        
    #Pos_final=transformation_library.from_matrix_to_pose(T_final)
    movegroup_library.go_to_pose_goal(pose)
    bool_exit=rospy.get_param("/CloseSystem")
    if bool_exit:
      return


def tf_subscriber(msg):
  global xyz_actual
  nul=0
  msg=msg.transforms[0]
  if(msg.child_frame_id!="tool0_controller"):
    return
  xyz_actual=[-msg.transform.translation.x,-msg.transform.translation.y,msg.transform.translation.z]
def clean_markers():
  global markerArray,marker_cont
  markerArray=MarkerArray()
  for cont_to_delete in range(0,10000):
    marker=Marker()
    marker.header.frame_id = "cameradepth_link"

    marker.ns = ""
    marker.id = cont_to_delete
    marker.action = 2

    markerArray.markers.append(marker)
  pub.publish(markerArray)
def force_listener(msg):
  global total_force
  x_2=msg.wrench.force.x*msg.wrench.force.x
  y_2=msg.wrench.force.y*msg.wrench.force.y
  z_2=msg.wrench.force.z*msg.wrench.force.z
  total_force=math.sqrt(x_2+y_2+z_2)
  if(total_force>0):
    print(total_force)
def main():
  global movegroup_library,bool_move_group_initialized,marker_cont,markerArray
  global pub

  define_all_initial_functions()
  rospy.Subscriber("/ft_sensor_topic",geometry_msgs.msg.WrenchStamped,force_listener)
  """
  pose=Pose()
  pose.position.x=1
  pose.position.y=0
  pose.position.z=0.1
  pose.orientation=transformation_library.from_euler_to_quaternion([0,0,0]).orientation
  
  movegroup_library.go_to_pose_cartesian(pose)"""
  while not rospy.is_shutdown():
    rospy.rostime.wallsleep(0.5)

if __name__ == '__main__':
  main()

