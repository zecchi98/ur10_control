
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Michael Lautman*/
#define InitialApproachPhase 1
#define ApproachPhase 2
#define SlidingPhase 3
#define PrepareSliding 4
#include <tf/tf.h>
#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <math.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <iostream>
#include <iostream>
#include <cstdlib>
#include <time.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <chrono>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "tf_conversions/tf_eigen.h"
using namespace std;
using namespace geometry_msgs;
using namespace moveit;
using namespace Eigen;
// C++ program to find Moore-Penrose inverse  matrix
bool joint_ready=false;
bool result_controller_received=false;
bool force_msg_received=false;
bool approach_confirmed=false;
sensor_msgs::JointState jointStateMsg;
moveit::core::RobotModelPtr kinematic_model;
moveit::core::JointModelGroup* joint_model_group;
moveit::core::RobotStatePtr kinematic_state;
moveit::planning_interface::MoveGroupInterface *move_group;
float total_force=0;
int phase=ApproachPhase;

double th_orient=0.0001;
double th_pos=0.001;
Affine3d ll;
Matrix3d ss;
Eigen::MatrixXd pinv(6,6);
Eigen::VectorXd e_dot(6,1),e_dot_dot(6,1),e_dot_iniziale(6,1),e_iniziale(6,1),e(6,1);
Eigen::MatrixXd q_dot(6,1),q_dot_dot(6,1),q_dot_iniziale(6,1),q_final(6,1),q_actual(6,1);


double final_position_x[3]={0.31,0.31,0.31};
double final_position_y[3]={0.35,0.35,0.37};
double final_position_z[3]={0.38,0.37,0.37};
int cont_posizioni=0;

float a_max=0.05,V_max=0.1;
int cont_allontanamento=0;
void joint_value_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  joint_ready=true;
  jointStateMsg=*msg;
}
void update_joints_value(){
  joint_ready=false;
  do{
  ros::spinOnce();
  }while(!joint_ready);

  kinematic_state->setVariableValues(jointStateMsg);
  joint_model_group = kinematic_model->getJointModelGroup("manipulator");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

  /*for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }*/

  q_actual(0,0)=joint_values[0];
  q_actual(1,0)=joint_values[1];
  q_actual(2,0)=joint_values[2];
  q_actual(3,0)=joint_values[3];
  q_actual(4,0)=joint_values[4];
  q_actual(5,0)=joint_values[5];
}
void from_pose_to_rpy(Pose p,float *rpy_vect){

  tf::Quaternion q(
          p.orientation.x,
          p.orientation.y,
          p.orientation.z,
          p.orientation.w);
    tf::Matrix3x3 m(q);
    double r0, p0, y0;

    m.getRPY(r0, p0, y0);
    rpy_vect[0]=r0;
    rpy_vect[1]=p0;
    rpy_vect[2]=y0;
    //cout<<r0<<" "<<p0<<" "<<y0<<endl;
}
Pose homo_to_pose(Affine3d homo){

  tf::Pose pose_tf;
  Pose pose;
  tf::poseEigenToTF(homo,pose_tf);
  tf::poseTFToMsg(pose_tf,pose);
  return pose;

}

trajectory_msgs::JointTrajectoryPoint elaborate_JointTrajectoryPoint(Eigen::MatrixXd q,Eigen::MatrixXd q_dot,Eigen::MatrixXd q_dot_dot,int index,float dt)
{

  trajectory_msgs::JointTrajectoryPoint j_traj;
  j_traj.positions={q(0,0),q(1,0),q(2,0),q(3,0),q(4,0),q(5,0)};
  j_traj.velocities={q_dot(0,0),q_dot(1,0),q_dot(2,0),q_dot(3,0),q_dot(4,0),q_dot(5,0)};
  j_traj.accelerations={q_dot_dot(0,0),q_dot_dot(1,0),q_dot_dot(2,0),q_dot_dot(3,0),q_dot_dot(4,0),q_dot_dot(5,0)};
  j_traj.effort={0,0,0,0,0,0}; 
  j_traj.time_from_start=ros::Duration(index*dt);
  return j_traj;
}
void elabora_cinematica_ee(float dt){
  float a,V0,t,X0,X,V;
  for(int i=0;i<6;i++){
    a=e_dot_dot(i,0);
    t=dt;
    V0=e_dot_iniziale(i,0);
    X0=e_iniziale(i,0);


    V=a*t+V0;
    X=X0+V0*t+(0.5)*a*t*t;
    
    e(i,0)=X;
    e_dot(i,0)=V;
    //cout<<"incrementing of:"<<X-X0<<endl;
    //cout<<endl<<endl;
  }
}
void elabora_cinematica_ee_v2(float dt){
  //In questa versione è nota la posizione ma non l'accelerazione
  //Viene comunque applicata una piccola modifica ad ee per rimanere attaccato all oggetto
  float a,V0,t,X0,X,V;
    for(int i=0;i<6;i++){

      t=dt;
      V0=e_dot_iniziale(i,0);
      X0=e_iniziale(i,0);
      X=e(i,0);



      a=((X-X0-V0*t)*2)/(t*t);
      
      if(a>a_max)
        a=a_max;
      if(a<-a_max)
        a=-a_max;
      
      V=V0+a*t;
      
      if(V>V_max)
        V=V_max;
      if(V<-V_max)
        V=-V_max;

      e(i,0)=X0+V0*t+0.5*a*t*t;

      //ROS_INFO_STREAM("a:"<<a);
      e_dot_dot(i,0)=a;
      e_dot(i,0)=V;
      //cout<<"incrementing of:"<<X-X0<<endl;
      //cout<<endl<<endl;
    }




}
void aggiorna_pseudo_inverse_jacobian(){

  update_joints_value();


  

  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(joint_model_group,
                               kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position, jacobian);
  pinv = jacobian.completeOrthogonalDecomposition().pseudoInverse();

  //ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
  //ROS_INFO_STREAM("pinv: \n" << pinv<< "\n");
}

void elabora_cinematica_q_and_trajectory(control_msgs::FollowJointTrajectoryActionGoal *goal_msg,int number_of_cycle,float dt){
  q_dot=pinv*e_dot;

  q_dot_dot=(q_dot-q_dot_iniziale)/dt;

  trajectory_msgs::JointTrajectoryPoint j_traj;

  q_final=q_actual+q_dot_iniziale*dt+0.5*q_dot_dot*dt*dt;
  
  j_traj=elaborate_JointTrajectoryPoint(q_final,q_dot,q_dot_dot,1,dt);
  goal_msg->goal.trajectory.points.push_back(j_traj);
  q_actual=q_final;
  q_dot_iniziale=q_dot;

  //q_final=q_actual+q_dot_iniziale*dt+0.5*q_dot_dot*dt*dt;
  //q_dot=q_dot_dot*dt+q_dot_iniziale;
}
Affine3d pose_to_homo(Pose pose){
  Affine3d homo;
  tf::Pose pose_tf;
  tf::poseMsgToTF(pose,pose_tf);
  tf::poseTFToEigen(pose_tf,homo);
  return homo;

}
Pose FindNextPose(Pose actualPose,Pose targetPose){
  Pose finalTarget=actualPose;

 // diffx=targetPose.position.x - actualPose.position.x;

  //QUESTA FUNZIONE NON FA NULLA

}
void controller_callback(const control_msgs::FollowJointTrajectoryActionResult& msg){
  result_controller_received=true;
}
void aggiorno_ee_con_valore_attuale(){

    Pose actual_pose=move_group->getCurrentPose().pose;
    float rpy_actual[3];
    from_pose_to_rpy(actual_pose,rpy_actual);


    //aggiorno ee
    e_iniziale(0,0)=actual_pose.position.x;
    e_iniziale(1,0)=actual_pose.position.y;
    e_iniziale(2,0)=actual_pose.position.z;
    e_iniziale(3,0)=rpy_actual[0];
    e_iniziale(4,0)=rpy_actual[1];
    e_iniziale(5,0)=rpy_actual[2];

    e(0,0)=actual_pose.position.x;
    e(1,0)=actual_pose.position.y;
    e(2,0)=actual_pose.position.z;
    e(3,0)=rpy_actual[0];
    e(4,0)=rpy_actual[1];
    e(5,0)=rpy_actual[2];
}
void wait_controller_to_finish(){
  do{
  ros::spinOnce();
  }while(!result_controller_received);
  result_controller_received=false;
}
void inizializza_matrici(){
  for(int i=0;i<6;i++){
    
    e_dot_dot(i,0)      =0;
    e_dot_iniziale(i,0) =0;
    e_dot(i,0)          =0;
    e_iniziale(i,0)     =0;
    e(i,0)              =0;
    q_dot(i,0)          =0;
    q_dot_dot(i,0)      =0;
    q_dot_iniziale(i,0) =0;
    q_final(i,0)        =0;
    q_actual(i,0)       =0;
  }
  aggiorno_ee_con_valore_attuale();
  aggiorna_pseudo_inverse_jacobian();
}
void aggiorna_matrici_iniziali_con_valori_attuali(){
  
  e_dot_iniziale =e_dot;
  q_dot_iniziale =q_dot;
  
}
void force_callback(const geometry_msgs::WrenchStamped& msg){

  force_msg_received=true;
  float x_2=msg.wrench.force.x*msg.wrench.force.x;
  float y_2=msg.wrench.force.y*msg.wrench.force.y;
  float z_2=msg.wrench.force.z*msg.wrench.force.z;
  total_force=sqrt(x_2+y_2+z_2);
}
void wait_force_callback(){
  force_msg_received=false;
  do{
  ros::spinOnce();
  }while(!force_msg_received);
  force_msg_received=false;
}
void decide_e_dot_dot(){
    float desired_force,k_fa_sliding,k_fa_approaching,max_abs_acc_value;

    max_abs_acc_value=0.1;
    k_fa_sliding=0.000001;
    k_fa_approaching=0.00001;
    desired_force=1500;
    float err_force=desired_force-total_force;

    if(total_force>100 && phase==ApproachPhase){
      phase=SlidingPhase;
      cout<<"Approccio raggiunto, mi riassesto";
      inizializza_matrici();
    }
    if(total_force<=100 && phase==SlidingPhase){
      inizializza_matrici();
      cout<<"Approccio mancato, mi riassesto";
      phase=ApproachPhase;
    }

    if(phase==ApproachPhase){
      e_dot_dot(0,0)=0.00;
      e_dot_dot(1,0)=err_force*k_fa_approaching;
    }
    if(phase==SlidingPhase){
      e_dot_dot(0,0)=0.01;
      e_dot_dot(1,0)=err_force*k_fa_sliding;
    }


    if(e_dot_dot(1,0)>max_abs_acc_value)
      e_dot_dot(1,0)=max_abs_acc_value;
    if(e_dot_dot(1,0)<-max_abs_acc_value)
      e_dot_dot(1,0)=-max_abs_acc_value;
    

    cout<<"total_force:"<<total_force<<endl;
    cout<<"Phase:"<<phase<<endl;
}
void decide_e(){
    Affine3d T_0_tool,T_tool_target,T_0_target;
    float trasl_x=0,trasl_y=0,trasl_z=0;
    T_0_tool=pose_to_homo(move_group->getCurrentPose().pose);


    if(total_force>30 && phase==ApproachPhase){
      phase=SlidingPhase;
      cout<<"Approccio raggiunto, mi riassesto";
      inizializza_matrici();
    }
    if(total_force<=30 && phase==SlidingPhase){
      inizializza_matrici();
      cout<<"Approccio mancato, mi riassesto";
      phase=ApproachPhase;
    }

    if(phase==ApproachPhase){
      trasl_x=0.01;
    }
    if(phase==SlidingPhase){
      trasl_y=0.03;
    }
    if(phase==PrepareSliding){
      
      //trasl_x=-0.01;
    }

    Vector3d transl(trasl_x,trasl_y,trasl_z);
    Matrix3d rot;
    Vector3d x_my_rot(1,0,0),y_my_rot(0,1,0),z_my_rot(1,0,0);
    rot.col(0)=x_my_rot;
    rot.col(1)=y_my_rot;
    rot.col(2)=z_my_rot;
    T_tool_target.translation()=transl;
    T_tool_target.linear()=rot;

    T_0_target=T_0_tool*T_tool_target;
    Pose target=homo_to_pose(T_0_target);

    e(0,0)=target.position.x;
    e(1,0)=target.position.y;
    e(2,0)=target.position.z;

    cout<<"total_force:"<<total_force<<endl;
    cout<<"Phase:"<<phase<<endl;



}
bool is_orientation_right(Pose targ,Pose pos){

  float diffx,diffy,diffz,diffw;
  diffx=(targ.orientation.x-pos.orientation.x)*(targ.orientation.x-pos.orientation.x);
  diffy=(targ.orientation.y-pos.orientation.y)*(targ.orientation.y-pos.orientation.y);
  diffz=(targ.orientation.z-pos.orientation.z)*(targ.orientation.z-pos.orientation.z);
  float val=sqrt(diffx+diffy+diffz);
  cout<<"Err orientamento:"<<val<<endl;
  if(val<th_orient && val>-th_orient)
  {
    return true;
  }
  else return false;

}
void decide_e_imposing_target(Pose *finalTargetPTR){
    Pose finalTarget=*finalTargetPTR;
    Affine3d T_0_tool,T_tool_target,T_0_target,T_penetration;
    float trasl_x=0,trasl_y=0,trasl_z=0,dposx,dposy,dposz;
    Pose ActualPose;
    ActualPose=move_group->getCurrentPose().pose;

    dposx=(finalTarget.position.x-ActualPose.position.x);
    dposy=(finalTarget.position.y-ActualPose.position.y);
    dposz=(finalTarget.position.z-ActualPose.position.z);




    T_0_tool=pose_to_homo(move_group->getCurrentPose().pose);

    if(total_force<=30 && phase==ApproachPhase){
    }
    if(total_force>30 && phase==ApproachPhase){
      phase=SlidingPhase;
      cout<<"Approccio raggiunto, mi riassesto"<<endl;
      inizializza_matrici();
    }
    if(total_force<=30 && phase==SlidingPhase){
      inizializza_matrici();
      cout<<"Approccio mancato, mi riassesto";
      phase=ApproachPhase;
    }

    if(phase==ApproachPhase){
      trasl_x=0.01;
    }
    if(phase==SlidingPhase){
      trasl_y=-dposy;
      trasl_z=-dposz;
      trasl_x=0.0001;
    }
    if(phase==PrepareSliding){
      
      trasl_x=-0.01;
    }


    {
    Vector3d transl(trasl_x,trasl_y,trasl_z);
    Matrix3d rot;
    Vector3d x_my_rot(1,0,0),y_my_rot(0,1,0),z_my_rot(1,0,0);
    rot.col(0)=x_my_rot;
    rot.col(1)=y_my_rot;
    rot.col(2)=z_my_rot;
    T_tool_target.translation()=transl;
    T_tool_target.linear()=rot;
    }


    T_0_target=T_0_tool*T_tool_target;



    Pose target=homo_to_pose(T_0_target);

    e(0,0)=target.position.x;
    e(1,0)=target.position.y;
    e(2,0)=target.position.z;
    e(3,0)=-3.14;
    e(4,0)=0;
    e(5,0)=1.11;

    cout<<"Transl_y:"<<trasl_y;
    cout<<" Transl_z:"<<trasl_z;
    //cout<<"Actual_pose_y:"<<ActualPose.position.y;
    cout<<" Total_force:"<<total_force<<endl;
    //cout<<"Phase:"<<phase<<endl;
    //ROS_INFO_STREAM("e: \n" << e<< "\n");
    
    if(dposy>-th_pos && dposy<th_pos && dposz>-th_pos && dposz<th_pos && is_orientation_right(ActualPose,finalTarget)){

      float posx=0,posy=0,posz=0;
      cont_posizioni++; 
      
      cout<<"Actual_Pose:"<<ActualPose.position.x<<" "<<ActualPose.position.y<<" "<<ActualPose.position.z<<endl;


      cout<<"Inserisci posx target:";
      cin>>posx;
      cout<<"Inserisci posy target:";
      cin>>posy;
      cout<<"Inserisci posz target:";
      cin>>posz;

      finalTargetPTR->position.x=posx;
      finalTargetPTR->position.y=posy;
      finalTargetPTR->position.z=posz;

    }
}

void check_phase(){
  if(phase==PrepareSliding){
    if(cont_allontanamento>=30){

      cont_allontanamento=0;
      phase=SlidingPhase;

    }
    else{
      cont_allontanamento=cont_allontanamento+1;
    }
  }
}
void print_debug(){

  ROS_INFO_STREAM("pinv: \n" << pinv<< "\n");
  ROS_INFO_STREAM("dt: 0.1\n");
  ROS_INFO_STREAM("e_iniziale: \n" << e_iniziale<< "\n");
  ROS_INFO_STREAM("e_dot_iniziale: \n" << e_dot_iniziale<< "\n");
  ROS_INFO_STREAM("e_dot_dot: \n" << e_dot_dot<< "\n");
  ROS_INFO_STREAM("e_dot: \n" << e_dot<< "\n");
  ROS_INFO_STREAM("e: \n" << e<< "\n");
  ROS_INFO_STREAM("q_dot_dot: \n" << q_dot_dot<< "\n");
  ROS_INFO_STREAM("q_dot: \n" << q_dot<< "\n");
  ROS_INFO_STREAM("q_actual: \n" << q_actual<< "\n");
  ROS_INFO_STREAM("q_final: \n" << q_final<< "\n");
  ROS_INFO_STREAM("q_dot_iniziale: \n" << q_dot_iniziale<< "\n");
  ROS_INFO("LA JACOBIANA VA CALCOLATA DI VOLTA IN VOLTAAA");
}
void state_machine(){
  //Fasi:
  //approccio_iniziale-> vado verso un target iniziale e continuo finchè non vado a sbattere contro il cubo
  //sliding_to_target-> vado verso un target tenendo d'occhio il sensore di forza
  //approaching_while_sliding -> riassesto l'approaching direction mentre faccio sliding

  switch (phase)
  {
  case InitialApproachPhase:
    /* code */
    break;
  
  default:
    break;
  }  



}
void metodo2(){
  


  ros::NodeHandle n;

  ros::Publisher pub_traj = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/arm_controller/follow_joint_trajectory/goal", 1000);
  ros::Subscriber sub = n.subscribe("/joint_states", 1000, joint_value_callback);
  ros::Subscriber sub2 = n.subscribe("/arm_controller/follow_joint_trajectory/result", 1000, controller_callback);
  ros::Subscriber sub_force = n.subscribe("/ft_sensor_topic", 1000, force_callback);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  string PLANNING_GROUP="manipulator";
  moveit::planning_interface::MoveGroupInterface move_group_temp(PLANNING_GROUP);
  move_group=&move_group_temp;
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model = robot_model_loader.getModel();

  //inizializing kinematic_state
  moveit::core::RobotStatePtr kinematic_state_temp(new moveit::core::RobotState(kinematic_model));
  kinematic_state=kinematic_state_temp;
  
  float dt;
  int number_of_cycle=1000;
  dt=0.05;




  inizializza_matrici();
  aggiorno_ee_con_valore_attuale();
  aggiorna_pseudo_inverse_jacobian();



  float posx=0,posy=0,posz=0;


  Pose finalTarget;
  finalTarget=move_group->getCurrentPose().pose;

  cout<<"Actual_Pose:"<<finalTarget.position.x<<" "<<finalTarget.position.y<<" "<<finalTarget.position.z<<endl;


  cout<<"Inserisci posx target:";
  cin>>posx;
  cout<<"Inserisci posy target:";
  cin>>posy;
  cout<<"Inserisci posz target:";
  cin>>posz;


  finalTarget.position.x=posx;
  finalTarget.position.y=posy;
  finalTarget.position.z=posz;
  print_debug();
  //finalTarget.position.x=final_position_x[cont_posizioni];
  //finalTarget.position.y=final_position_y[cont_posizioni];
  //finalTarget.position.z=final_position_z[cont_posizioni];
  
  

  for(int iii=0;iii<number_of_cycle;iii++){

    
    control_msgs::FollowJointTrajectoryActionGoal goal_msg;
    goal_msg.goal_id.id=to_string(ros::Time::now().toSec());
    goal_msg.goal.trajectory.joint_names={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    goal_msg.header.frame_id="base";
    goal_msg.goal.trajectory.header.frame_id="base";
    

    
    //wait_force_callback();


    aggiorna_pseudo_inverse_jacobian();

    aggiorno_ee_con_valore_attuale();
    
    decide_e_imposing_target(&finalTarget);

    state_machine();

    elabora_cinematica_ee_v2(dt);

    elabora_cinematica_q_and_trajectory(&goal_msg,0,dt);
    
    //print_debug();
    
    pub_traj.publish(goal_msg);

    //wait_controller_to_finish();
    aggiorna_matrici_iniziali_con_valori_attuali();
    //check_phase();
    //cout<<"Aspetta input()";
    //char s;
    //cin>>s;

  }

}
void metodo1(){


  ros::NodeHandle n;

  ros::Publisher pub_traj = n.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/arm_controller/follow_joint_trajectory/goal", 1000);
  ros::Subscriber sub = n.subscribe("/joint_states", 1000, joint_value_callback);
  ros::Subscriber sub2 = n.subscribe("/arm_controller/follow_joint_trajectory/result", 1000, controller_callback);
  ros::Subscriber sub_force = n.subscribe("/ft_sensor_topic", 1000, force_callback);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  string PLANNING_GROUP="manipulator";
  moveit::planning_interface::MoveGroupInterface move_group_temp(PLANNING_GROUP);
  move_group=&move_group_temp;
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  kinematic_model = robot_model_loader.getModel();

  //inizializing kinematic_state
  moveit::core::RobotStatePtr kinematic_state_temp(new moveit::core::RobotState(kinematic_model));
  kinematic_state=kinematic_state_temp;
  
  float dt;
  int number_of_cycle=50;
  dt=0.1;
  inizializza_matrici();
  for(int iii=0;iii<number_of_cycle;iii++){
    control_msgs::FollowJointTrajectoryActionGoal goal_msg;
    goal_msg.goal_id.id=to_string(ros::Time::now().toSec());
    goal_msg.goal.trajectory.joint_names={"shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"};
    goal_msg.header.frame_id="base";
    goal_msg.goal.trajectory.header.frame_id="base";
    

    
    wait_force_callback();

    decide_e_dot_dot();

    aggiorna_pseudo_inverse_jacobian();

    aggiorno_ee_con_valore_attuale();
    
    elabora_cinematica_ee(dt);

    elabora_cinematica_q_and_trajectory(&goal_msg,0,dt);
    
    pub_traj.publish(goal_msg);

    //wait_controller_to_finish();
    aggiorna_matrici_iniziali_con_valori_attuali();
  }
}
int main(int argc, char** argv)
{

  ros::init(argc, argv, "robot_model_and_robot_state_tutorial");

  metodo2();
  
  //cout<<goal_msg;
  
  ros::shutdown();
  return 0;
}