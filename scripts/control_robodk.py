#!/usr/bin/env python
#fold all: ctrl + k + 0
#unfold all: ctrl + k + j

#To start robodk: ./Robodk-Start.sh
import copy
import math
from shutil import move
import sys
import time
from logging import root, setLoggerClass
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
from robodk import robolink, robomath      # import the robotics toolbox
from scipy.spatial.transform import Rotation 


def conti_utili_forse_un_giorno():

    std_pose=transformation_library.from_matrix_to_pose(matrix)

    print("Std pose:")
    print(std_pose)
    rotation_matrix=transformation_library.Rotation_matrix_of_Affine_matrix(matrix)

    print("rotation_matrix:")
    print(rotation_matrix)

    t=[rotation_matrix[0],rotation_matrix[1],rotation_matrix[2]]
    r = Rotation.from_dcm(rotation_matrix)
    
    print(r)
    print(r.as_euler('zyx', degrees=True))
    print(r.as_euler('zyx', degrees=False))
    print(r.as_euler('xyz', degrees=True))
    print(r.as_euler('xyz', degrees=False))

    eul=r.as_euler('zxy', degrees=True)

    eul_rad=transformation_library.rot2eul(rotation_matrix)
    print("eul_rad:")
    print(eul_rad)

    rpy_rad=transformation_library.rpy_from_quat(std_pose.orientation)
    print("rpy_rad:")
    print(rpy_rad)
    
    rpy_grad=transformation_library.rpy_from_rad_to_grad(rpy_rad)
    print("rpy_grad:")
    print(rpy_grad)

    angles=eul
    posedk=[std_pose.position.x,std_pose.position.y,std_pose.position.z,angles[0],angles[1],angles[2]]


    print("posedk:")
    print(posedk)

flagMiddlePanelCreated=False
bool_exit=False
class Transformation_class():
  def __init__(self):
        null=0
  def rad_to_grad(self,angle):
    return angle*180/math.pi
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
  def rpy_from_rad_to_grad(self,rpy_vect):
    return [self.rad_to_grad(rpy_vect[0]),self.rad_to_grad(rpy_vect[1]),self.rad_to_grad(rpy_vect[2])]
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
 
class RoboDK_class():
  def __init__(self):
    self.RDK = robolink.Robolink()
    self.robot = self.RDK.Item('UR10')      # retrieve the robot by name
    print("Robodk initialized")
  def prova_movimento(self):  
    RDK=self.RDK
    robot=self.robot
    #robot.setJoints([0,0,0,0,0,0])      # set all robot axes to zero
    time.sleep(2)
    target = RDK.Item('Home')         # retrieve the Target item
    robot.MoveJ(target)                 # move the robot to the target

    target = RDK.Item('Approach')         # retrieve the Target item
    robot.MoveJ(target)                 # move the robot to the target
    approach = target.Pose()*robomath.transl(0,0,-100)
    print(approach)
    robot.MoveL(approach) #Linear move to approach
  def go_to_posedk(self,matrix_dk):
    RDK=self.RDK
    robot=self.robot

    solution=self.robot.SolveIK(matrix_dk)
    if not self.check_if_matrix_is_a_valid_pose(matrix_dk):
      return False
    robot.MoveJ(matrix_dk) #Linear move to approachs
    return True
  def from_std_pose_to_robotdk_pose(self,std_pose):
    #std_pose significa la pose classica usata per moveit
    RDK=self.RDK
    robot=self.robot
    
    orientation_array=transformation_library.rpy_from_quat(std_pose.orientation)
    orientation_array[0]=transformation_library.rad_to_grad(orientation_array[0])
    orientation_array[1]=transformation_library.rad_to_grad(orientation_array[1])
    orientation_array[2]=transformation_library.rad_to_grad(orientation_array[2])
    pose=robomath.Pose(std_pose.position.x,std_pose.position.y,std_pose.position.z,
                      orientation_array[0],orientation_array[1],orientation_array[2])
    return pose
  def get_actual_matrix(self):
    RDK=self.RDK
    robot=self.robot
     # retrieve the robot by name
    return robot.Pose()
  def from_matrix_to_posedk_vect(self,matrix):
    
    pose_rad=robomath.Pose_2_UR(matrix)
    orientation_rad=[pose_rad[3],pose_rad[4],pose_rad[5]] 
    orientation_grad=transformation_library.rpy_from_rad_to_grad(orientation_rad)

    pose_grad=pose_rad
    pose_grad[3]=orientation_grad[0]
    pose_grad[4]=orientation_grad[1]
    pose_grad[5]=orientation_grad[2]
    #print("Pose_grad:")
    #print(pose_grad)
    return pose_grad
  def from_posedk_vect_to_matrix(self,posedk):
    return robomath.Pose(posedk[0],posedk[1],posedk[2],posedk[3],posedk[4],posedk[5])
  def transl(self,axes,step=100):
    if(axes=="x"):
      target=self.get_actual_matrix()*robomath.transl(step,0,0)
    if(axes=="y"):
      target=self.get_actual_matrix()*robomath.transl(0,step,0)
    if(axes=="z"):
      target=self.get_actual_matrix()*robomath.transl(0,0,step)

    self.go_to_posedk(target)
  def rot(self,axes,step=10):
    #step is in grads
    step=transformation_library.grad_to_rad(step)
    if(axes=="x"):
      target=self.get_actual_matrix()*robomath.rotx(step)
    if(axes=="y"):
      target=self.get_actual_matrix()*robomath.roty(step)
    if(axes=="z"):
      target=self.get_actual_matrix()*robomath.rotz(step)
    self.go_to_posedk(target)
  def check_if_matrix_is_a_valid_pose(self,matrix):

    solution=self.robot.SolveIK(matrix)
    for x in solution:
      if len(x)==1 :
        print("Errore, out of bounds")
        return False
    return True
  def get_actual_joint(self):
    j=self.robot.Joints()
    return j

#std functions
def callback_user_interface(msg):
  global bool_message_from_user,msg_from_user
  msg_from_user=UserInterfaceRequest()
  msg_from_user.modality=msg.modality
  msg_from_user.second_information=msg.second_information
  msg_from_user.target_pose=msg.target_pose
  msg_from_user.target_joints=msg.target_joints
  bool_message_from_user=True
  return UserInterfaceResponse()
def handle_user_request():
  global bool_message_from_user,bool_exit
  bool_message_from_user=False
  print("Msg received:"+msg_from_user.modality)
  if msg_from_user.modality=="automazione_go_pos_iniziale":
    movimenti_base_library.go_to_initial_position()
  if msg_from_user.modality=="controlla_gripper":
    move_gripper(msg_from_user.second_information)
  if msg_from_user.modality=="joystick":
    handle_joystick_input(msg_from_user.second_information)
  if msg_from_user.modality=="Stampa_pose_robot":
    movegroup_library.Stampa_Info_Robot()
  if msg_from_user.modality=="stop_trajectory":
    movegroup_library.FermaRobot()
  if msg_from_user.modality=="salva_aruco":
    aruco_library.save_visible_arucos()    
  if msg_from_user.modality=="turn_on_off_camera":
    comunication_object.call_cv_service("turn_on_off_camera","")
  if msg_from_user.modality=="exit":
    bool_exit=True
    comunication_object.call_cv_service("exit","")
  if msg_from_user.modality=="pose_randomOrientation":
     #
     # movegroup_library.go_to_pose_goal(msg_from_user.target_pose,True)
     print("deprecated")
  if msg_from_user.modality=="pose":
    pose_dk=robodk_library.from_std_pose_to_robotdk_pose(msg_from_user.target_pose)
    robodk_library.go_to_posedk(pose_dk)
def handle_joystick_input(input):
  global joystick_verso_rotazione,joystick_translation_step,joystick_angle_step,bool_message_from_user
  #q w -> asse x
  #a s -> asse y
  #z x -> asse z
  #o p -> rotazione asse x
  #k l -> rotazione asse y
  #n m -> rotazione asse z
  print(input)
  #if bool_pose_move==True allora si dovra effettuare un movimento go_to_pose
  #if bool_joint_move==True allora si dovra effettuare un movimento go_to_joint_state
  bool_message_from_user=True
  bool_pose_move=False
  bool_joint_move=False
  
  actual_matrix=robodk_library.get_actual_matrix()
  actual_pose=robodk_library.from_matrix_to_posedk_vect(actual_matrix)
  target_pose=actual_pose

  #print("Actual_matrix:")
  #print(actual_matrix)
  #print("Actual_pose:")
  #print(actual_pose)


  if(input=="q"):
    robodk_library.transl("x",joystick_translation_step)
  if(input=="w"):
    robodk_library.transl("x",-joystick_translation_step)
  if(input=="a"):
    robodk_library.transl("y",joystick_translation_step)
  if(input=="s"):
    robodk_library.transl("y",-joystick_translation_step)
  if(input=="z"):
    robodk_library.transl("z",joystick_translation_step)
  if(input=="x"):
    robodk_library.transl("z",-joystick_translation_step)
  if(input=="o"):
    robodk_library.rot("x",joystick_angle_step)
  if(input=="p"):
    robodk_library.rot("x",-joystick_angle_step)
  if(input=="k"):
    robodk_library.rot("y",joystick_angle_step)
  if(input=="l"):
    robodk_library.rot("y",-joystick_angle_step)
  if(input=="n"):
    robodk_library.rot("z",joystick_angle_step)
  if(input=="m"):
    robodk_library.rot("z",-joystick_angle_step)



  if(input>="1" and input<="6"):
    #trasforma char in numero tramite ascii
    num=ord(input)-ord("1")+1
    angle_grad_step=transformation_library.rad_to_grad(joystick_angle_step)
    movegroup_library.ruota_giunto(num-1,joystick_angle_step*joystick_verso_rotazione)

  if(input=="0"):
    joystick_verso_rotazione=(-1)*(joystick_verso_rotazione)


  #print(T_tool_camera_gazebo.Affine_matrix)
def define_all_initial_functions():
  global comunication_object,transformation_library,robodk_library
  global joystick_verso_rotazione,joystick_angle_step,joystick_translation_step,bool_message_from_user
  
  rospy.init_node('state_machine', anonymous=True)
  robodk_library=RoboDK_class()
  comunication_object=Comunication_class()
  transformation_library=Transformation_class()

  bool_message_from_user=False
  
  #joystick
  joystick_verso_rotazione=1
  joystick_translation_step=50
  joystick_angle_step=5

  robodk_library.RDK.setCollisionActive(1)
  rospy.Service('user_interface_serv', UserInterface, callback_user_interface)
  
def test_funzionamento_sistema():
  robodk_library.go_to_posedk()
def prova():
  for i in range(1,10):
    j=robodk_library.get_actual_joint()
    target=robodk_library.get_actual_matrix()*robomath.transl(0,-300,0)
    ret=robodk_library.robot.MoveL_Test(j,target,1)
    print(ret)
    #time.sleep(0.5)
  print("ok")
def main():
  define_all_initial_functions()
  #prova()
  #test_funzionamento_sistema()    
  try:
    while (not rospy.core.is_shutdown()) and (not bool_exit):


        
        rospy.rostime.wallsleep(0.5)
        if bool_message_from_user:
          handle_user_request()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

  except bool_exit==True:
      return
if __name__ == '__main__':
  main()
