#!/usr/bin/env python
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton
from PyQt5 import uic, QtWidgets,QtGui
import rospy
from std_msgs.msg import String
from ur10_control.srv import UserInterface,UserInterfaceRequest,aruco_service,aruco_serviceRequest
from geometry_msgs.msg import Pose
from tf import transformations
import rospkg
import math
rospack = rospkg.RosPack()

pathTopkg=rospack.get_path('ur10_control')
def spegni_tutto():
    app.closeAllWindows()   
    try:
            modality='exit'
            target_pose=Pose()
            target_joints=[0,0,0,0,0,0]
            second_information='null'
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
    except rospy.ServiceException as e:
            print("Errore")

class Manual_pose(QWidget):
    global widget,window_mp,window_joystick,window_main,serv,app
    def __init__(self):
        super(Manual_pose,self).__init__()
        uic.loadUi(pathTopkg + '/QT/manual_pose.ui', self)
        
    def pub(self):
        pub = rospy.Publisher('chatter', String, queue_size=10)
        hello_str = "hello world %s" % rospy.get_time()
        pub.publish(hello_str)
    def service(self):
        rospy.wait_for_service('/user_interface_serv')
        try:
            resp1 = serv('joint')
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    def pB_esegui(self):
        window_mp.setEnabled(False)
        app.processEvents()
        if self.checkRandomOrientation.isChecked():
            modality='pose_randomOrientation'
        else:
            modality='pose'

        target_joints=[0,0,0,0,0,0]
        target_pose=Pose()

        quaternion = transformations.quaternion_from_euler(math.radians(float(self.mpRoll.text())), math.radians(float(self.mpPitch.text())), math.radians(float(self.mpYaw.text())))
        target_pose.position.x=float(self.mpX.text())
        target_pose.position.y=float(self.mpY.text())
        target_pose.position.z=float(self.mpZ.text())
        target_pose.orientation.x = quaternion[0]
        target_pose.orientation.y = quaternion[1]
        target_pose.orientation.z = quaternion[2]
        target_pose.orientation.w = quaternion[3]
        second_information='null'

        try:
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
        except rospy.ServiceException as e:
            self.l_comunicazione.setText('Errore:'+ e)

        window_mp.setEnabled(True)
    def spegni_console(self):
        spegni_tutto()
    
    # def fRandomOrientationcheck(self):
    #     global boolRandomOrientation
    #     self.l_comunicazione.setText('CheckBox modification')
    #     if self.checkRandomOrientation.isChecked():
    #         boolRandomOrientation=True
    #     else:
    #         boolRandomOrientation=False
    def pB_setJoints(self):
        try:
            modality='joints'
            second_information='null'
            target_pose=Pose()
            target_joints=[float(self.mpJ_1.text()),float(self.mpJ_2.text()),float(self.mpJ_3.text()),float(self.mpJ_4.text()),float(self.mpJ_5.text()),float(self.mpJ_6.text())]
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
        except rospy.ServiceException as e:
            self.l_comunicazione.setText('Errore:'+ e)
    def goToJoystick(self):
        widget.removeWidget(window_mp)
        widget.addWidget(window_joystick)
    def goToMainMenu(self):
        widget.removeWidget(window_mp)
        widget.addWidget(window_main)
class Screen_Joystick(QWidget):
    global widget,window_mp,window_joystick,window_main
    def __init__(self):
        super(Screen_Joystick,self).__init__()
        uic.loadUi(pathTopkg + '/QT/screen_joystick.ui', self)
    def joystick_facsimile(self,number_joint):
        try:
            modality='joystick'
            target_pose=Pose()

            step=float(self.joystick_line_gradi.text())
            target_joints=[step,0,0,0,0,0]
            second_information=number_joint
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
        except rospy.ServiceException as e:
            self.l_comunicazione.setText('Errore:'+ e)
    def joystick_ee_facsimile(self,type_of_ee):
        try:
            modality='joystick'
            target_pose=Pose()
            step_ee_slider=self.step_ee_slider.value()
            target_joints=[step_ee_slider,0,0,0,0,0]
            second_information=type_of_ee
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
        except rospy.ServiceException as e:
            self.l_comunicazione.setText('Errore:'+ e)   
    def joystick_joint1(self):
        self.joystick_facsimile("1")
    def joystick_joint2(self):
        self.joystick_facsimile("2")
    def joystick_joint3(self):
        self.joystick_facsimile("3")
    def joystick_joint4(self):
        self.joystick_facsimile("4")
    def joystick_joint5(self):
        self.joystick_facsimile("5")
    def joystick_joint6(self):
        self.joystick_facsimile("6")
    def joystick_x_clock(self):
        self.joystick_ee_facsimile("q")
    def joystick_x_anticlock(self):
        self.joystick_ee_facsimile("w")
    def joystick_y_clock(self):
        self.joystick_ee_facsimile("a")
    def joystick_y_anticlock(self):
        self.joystick_ee_facsimile("s")
    def joystick_z_clock(self):
        self.joystick_ee_facsimile("z")
    def joystick_z_anticlock(self):
        self.joystick_ee_facsimile("x")
    def joystick_roll_clock(self):
        self.joystick_ee_facsimile("o")
    def joystick_roll_anticlock(self):
        self.joystick_ee_facsimile("p")
    def joystick_pitch_clock(self):
        self.joystick_ee_facsimile("k")
    def joystick_pitch_anticlock(self):
        self.joystick_ee_facsimile("l")
    def joystick_yaw_clock(self):
        self.joystick_ee_facsimile("n")
    def joystick_yaw_anticlock(self):
        self.joystick_ee_facsimile("m")
    def joystick_stop(self):
        try:
            modality='joystick_stop'
            target_pose=Pose()
            target_joints=[0,0,0,0,0,0]
            second_information='null'
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
        except rospy.ServiceException as e:
            self.l_comunicazione.setText('Errore:'+ e)
    def spegni_console(self):
        spegni_tutto()
    def joystick_moveJoints(self):
        window_joystick.setEnabled(False)
        app.processEvents()

        step=float(self.joystick_line_gradi.text())
        target_joints=[0,0,0,0,0,0]
        target_pose=Pose()
        modality='joystick_Joints'
        second_information='null'

        if self.joystick_check_sensoRotazione.isChecked():
            if self.joystick_check_Joint1.isChecked():
                target_joints[0]=-step
            if self.joystick_check_Joint2.isChecked():
                target_joints[1]=-step
            if self.joystick_check_Joint3.isChecked():
                target_joints[2]=-step
            if self.joystick_check_Joint4.isChecked():
                target_joints[3]=-step
            if self.joystick_check_Joint5.isChecked():
                target_joints[4]=-step
            if self.joystick_check_Joint6.isChecked():
                target_joints[5]=-step
        else:
            if self.joystick_check_Joint1.isChecked():
                target_joints[0]=step
            if self.joystick_check_Joint2.isChecked():
                target_joints[1]=step
            if self.joystick_check_Joint3.isChecked():
                target_joints[2]=step
            if self.joystick_check_Joint4.isChecked():
                target_joints[3]=step
            if self.joystick_check_Joint5.isChecked():
                target_joints[4]=step
            if self.joystick_check_Joint6.isChecked():
                target_joints[5]=step      
        self.l_comunicazione.setText(str(target_joints[0])+ ' '+ str(target_joints[1])+ ' '+  str(target_joints[2])+ ' '+  str(target_joints[3])+ ' '+  str(target_joints[4])+ ' '+  str(target_joints[5]))
        try:
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
        except rospy.ServiceException as e:
            self.l_comunicazione.setText('Errore:'+ e)

        window_joystick.setEnabled(True)
    def goToManualPose(self):
        widget.removeWidget(window_joystick)
        widget.addWidget(window_mp)
    def goToMainMenu(self):
        widget.removeWidget(window_joystick)
        widget.addWidget(window_main)
    def salva_point_in_cloud(self):
        target_joints=[0,0,0,0,0,0] 
        target_pose=Pose()
        modality='salva_point_in_cloud'
        second_information=""
        try:
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
        except rospy.ServiceException as e:   
            print('Errore:'+ e)      

class Screen_Main(QWidget):
    global widget,window_mp,window_joystick,window_main,window_automazione
    def __init__(self):
        super(Screen_Main,self).__init__()
        uic.loadUi(pathTopkg + '/QT/screen_main.ui', self)
    
    def camera_on_off(self):
        try:
            modality='turn_on_off_camera'
            target_pose=Pose()
            target_joints=[0,0,0,0,0,0]
            second_information='null'
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
        except rospy.ServiceException as e:
            print("errore")
            print(e)
        
    def spegni_console(self):
        spegni_tutto()
    def goToManualPose(self):
        widget.removeWidget(window_main)
        widget.addWidget(window_mp)
    def goToJoystick(self):
        widget.removeWidget(window_main)
        widget.addWidget(window_joystick)
    def goToAutomazione(self):
        widget.removeWidget(window_main)
        widget.addWidget(window_automazione)
class Screen_Automazione(QWidget):
    global widget,window_mp,window_joystick,window_main,window_automazione
    def __init__(self):
        super(Screen_Automazione,self).__init__()
        uic.loadUi(pathTopkg + '/QT/screen_automazione.ui', self)
    
    def spegni_console(self):
        spegni_tutto()
    def goToMainMenu(self):
        widget.removeWidget(window_automazione)
        widget.addWidget(window_main)
    def FollowTrajectory(self):
        target_joints=[0,0,0,0,0,0] 
        target_pose=Pose()
        modality='FollowTrajectory'
        second_information='null'
        try:
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
        except rospy.ServiceException as e:
            self.l_comunicazione.setText('Errore:'+ e)
    def ruota_e_cerca_pannello(self):
        target_joints=[0,0,0,0,0,0] 
        target_pose=Pose()
        modality='ruota_e_cerca_pannello'
        second_information='null'
        try:
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
        except rospy.ServiceException as e:
            self.l_comunicazione.setText('Errore:'+ e)
    def PlotPoints(self):
        target_joints=[0,0,0,0,0,0]
        target_pose=Pose()
        modality='PlotPoints'
        second_information='null'
        try:
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
        except rospy.ServiceException as e:
            self.l_comunicazione.setText('Errore:'+ e)

    def automazione_pannello_MoveToSelectedAruco(self):
        target_joints=[0,0,0,0,0,0] 
        target_pose=Pose()
        modality='automazione_pannello_MoveToSelectedAruco'
        second_information='null'
        try:
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
        except rospy.ServiceException as e:
            self.l_comunicazione.setText('Errore:'+ e)
    def ChangeDistanceFromObj(self):
        target_joints=[0,0,0,0,0,0] 
        target_pose=Pose()
        modality='ChangeDistanceFromObj'
        second_information=self.select_aruco_value.text()
        try:
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
        except rospy.ServiceException as e:
            self.l_comunicazione.setText('Errore:'+ e)
    def elaboraManualCloud(self):
        target_joints=[0,0,0,0,0,0] 
        target_pose=Pose()
        modality='elaboraManualCloud'
        second_information=""
        try:
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
        except rospy.ServiceException as e:   
            print('Errore:'+ e)      




    def stop_traj_clicked(self):
        modality='stop_trajectory'
        second_information=''
        try:
            msg=aruco_serviceRequest(modality,second_information)
            resp1 = bridge_serv(msg)
        except rospy.ServiceException as e:
            self.l_comunicazione.setText('Errore:'+ e)
    def centra_aruco(self):
        target_joints=[0,0,0,0,0,0]
        target_pose=Pose()
        modality='centra_aruco'
        second_information=self.select_aruco_value.text()
        try:
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
        except rospy.ServiceException as e:
            self.l_comunicazione.setText('Errore:'+ e)


    def centra_aruco_and_zoom_in(self):
        target_joints=[0,0,0,0,0,0]
        target_pose=Pose()
        modality='centra_aruco_and_zoom_in'
        second_information=self.select_aruco_value.text()
        try:
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
        except rospy.ServiceException as e:
            self.l_comunicazione.setText('Errore:'+ e)
    def centra_aruco_and_zoom_out(self):
        target_joints=[0,0,0,0,0,0]
        target_pose=Pose()
        modality='centra_aruco_and_zoom_out'
        second_information=self.select_aruco_value.text()
        try:
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
        except rospy.ServiceException as e:
            self.l_comunicazione.setText('Errore:'+ e)
    def automazione_go_pos_iniziale(self):
        target_joints=[0,0,0,0,0,0]
        target_pose=Pose()
        modality='automazione_go_pos_iniziale'
        second_information=''
        try:
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
        except rospy.ServiceException as e:
            self.l_comunicazione.setText('Errore:'+ e)
    def automazione_debug1(self):
        target_joints=[0,0,0,0,0,0]
        target_pose=Pose()
        modality='automazione_debug1'
        second_information=''
        try:
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
        except rospy.ServiceException as e:
            self.l_comunicazione.setText('Errore:'+ e)
    def automazione_debug2(self):
        target_joints=[0,0,0,0,0,0]
        target_pose=Pose()
        modality='automazione_debug2'
        second_information=''
        try:
            msg=UserInterfaceRequest(modality,second_information,target_pose,target_joints)
            resp1 = serv(msg)
        except rospy.ServiceException as e:
            self.l_comunicazione.setText('Errore:'+ e)
#MAIN
def main_code():
    global widget,window_mp,window_joystick,window_main,serv,bridge_serv,app,window_automazione
    rospy.init_node('user_interface', anonymous=True)
    serv = rospy.ServiceProxy('/user_interface_serv', UserInterface)
    bridge_serv= rospy.ServiceProxy('/aruco_modality', aruco_service)
    app=QApplication([])

    widget=QtWidgets.QStackedWidget()
    window_joystick=Screen_Joystick()
    window_mp=Manual_pose()
    window_main=Screen_Main()
    window_automazione=Screen_Automazione()

    widget.addWidget(window_main)
    widget.setFixedWidth(700)
    widget.setFixedHeight(600)

    widget.show()
    app.exec_()


if __name__ == '__main__':
    try:
        main_code()
    except rospy.ROSInterruptException:
        pass
