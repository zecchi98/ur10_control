#!/usr/bin/env python

#from tkinter.tix import Tree
import rospy
import numpy as np

import sys
import os
from sensor_msgs.msg import Image as sensImg
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Empty
import cv2 as cv
import cv2.aruco as aruco
from cv_bridge import CvBridge
from ur10_control.srv import *
def handler_cv_server(msg):
    global bool_exit,bool_camera_on
    if msg.message=="exit":
        bool_exit=True
    if msg.message=="turn_on_off_camera":
        bool_camera_on=not (bool_camera_on)
    return cv_serverResponse()

def loadCameraParam(myCam):
    ##
    #\brief this function will load camera parameters
    #@param myCam Topic of the camera
    #@return No return
    global cameraMatr,cameraDistCoefs,cameraFocLen
    global cameraDistCoefs
    global cameraFocLen
    
    print('loading camera parameters...')
    cameraInfoMsg=rospy.wait_for_message(myCam+'/camera_info',CameraInfo)
    cameraMatr=np.reshape(cameraInfoMsg.K,[3,3])
    cameraDistCoefs=cameraInfoMsg.D
    cameraFocLen=np.mean([np.ravel(cameraMatr[0])[0],np.ravel(cameraMatr[1])[1]])
def callbackRaw(raw_img):
    ##
    #\brief This function is the callback which is subscribed to the raw channel of the image. It will detect arucos and in case it is a new one it will save it and check for victory.
    #@param raw_img It is a sensor_msgs of type Image which contain the image we will analyze
    #@return No return

    global bool_exit

    #if this bool is a true than it close everything
    if bool_exit:
        cv.destroyAllWindows()
        os._exit(os.EX_OK)

    #Here the detection will be computed based on the dictionary
    cv_image=bridge.imgmsg_to_cv2(raw_img, desired_encoding='passthrough')
    cv_gray=cv.cvtColor(cv_image,cv.COLOR_RGB2GRAY)
    detCorners, detIds, _ = aruco.detectMarkers(cv_gray, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
        
    # Check if at least one marker has been found
    if detIds is not None and len(detIds) >= 1:

        #Draw the borders around the aruco
        detAruImg = aruco.drawDetectedMarkers(cv_image.copy(), detCorners, borderColor=(0, 255, 0))

        #Foreach aruco found it will work on it
        for mId, aruPoints in zip(detIds, detCorners):
            nulla=0
    else:
        detAruImg=cv_image.copy()#
    if bool_camera_on:
        cv.imshow('detected markers',detAruImg)
    else:
        cv.destroyAllWindows()
    key = cv.waitKey(12) & 0xFF
def main():
    ##
    #\brief Here we initialize some variables and thanks to spin we wait for callbackRaw
    global bridge,bool_exit,ARUCO_PARAMETERS,ARUCO_DICT,bool_camera_on

    bool_exit=False
    bridge=CvBridge()

    #Camera topics
    raw_topic="/camera/color/image_raw"
    myCamera="/camera/color"

    bool_camera_on=rospy.get_param("camera_on")
    
    #rospy initialization
    rospy.init_node('camera_listener', anonymous=True)
    #Subscriber initialization (image callback)
    rospy.Subscriber(raw_topic,sensImg,callbackRaw,queue_size = 1)
    rospy.Service("/cv_server",cv_server,handler_cv_server)
    
    ARUCO_PARAMETERS = aruco.DetectorParameters_create()
    ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)


    print('connecting to:'+myCamera+'...')
    loadCameraParam(myCamera)
    print('ready')

    #Thanks to spin we will wait callbacks
    try:
        rospy.spin()
    except KeyboardInterrupt:#
        print('Closing')
    cv.destroyAllWindows()


if __name__ == '__main__':
    main()




