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

#from pytest import Mark, mark

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

bool_robodk_enabled=False
flagMiddlePanelCreated=False
bool_exit=False
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
class Collision_Box():
    def __init__(self):
        self.box_pose = PoseStamped()
        self.box_size = np.zeros(3)
        self.box_name = String()  

 
def define_all_initial_functions():
  global movegroup_library,comunication_object,transformation_library,movimenti_base_library,aruco_library
  global joystick_verso_rotazione,joystick_angle_step,joystick_translation_step,bool_message_from_user,markerArray,pub,marker_cont
  global MaxTranslationStep,MaxRotationStep
  movegroup_library = Move_group_class()
  transformation_library=Transformation_class()
  

def move_robot_to_cube_pose():
    print("Inizio movimento")
    #Questa funzione muove il robot nella posizione conforme a leggere il cubo
    nul=0
    joint_vet=[0,0,0,0,0,0]
    joint_vet[0]=transformation_library.grad_to_rad(34)
    joint_vet[1]=transformation_library.grad_to_rad(-86)
    joint_vet[2]=transformation_library.grad_to_rad(146)
    joint_vet[3]=transformation_library.grad_to_rad(120)
    joint_vet[4]=transformation_library.grad_to_rad(-60)
    joint_vet[5]=transformation_library.grad_to_rad(-179)
    movegroup_library.go_to_joint_state(joint_vet)
    print("Fine movimento")
def main():
  define_all_initial_functions()
  move_robot_to_cube_pose()
  print("adding collision")
  """collision_box=Collision_Box()
  collision_box.box_name=str(1)
  collision_box.box_pose.header.frame_id = "cameradepth_link"
  collision_box.box_size[0]=0.1
  collision_box.box_size[1]=0.1
  collision_box.box_size[2]=0.1
  collision_box.box_pose.pose.position.x=1.14
  collision_box.box_pose.pose.position.y=0.26
  collision_box.box_pose.pose.position.z=0.35
  pose_quat=transformation_library.from_euler_to_quaternion([0,0,0])
  collision_box.box_pose.pose.orientation=pose_quat.orientation
  print(collision_box.box_pose)
  print(collision_box.box_size)
  print(collision_box.box_name)
  movegroup_library.add_box(collision_box)"""
if __name__ == '__main__':
  main()
