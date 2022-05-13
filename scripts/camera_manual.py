#!/usr/bin/env python

#from tkinter.tix import Tree
from cv2 import imshow
import rospy
import numpy as np
import cv2
import sys
import os
from sensor_msgs.msg import Image as sensImg
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Empty
import cv2 as cv
import cv2.aruco as aruco
from cv_bridge import CvBridge
from ur10_control.srv import *
import rospkg
import time
import numpy as np
from matplotlib import pyplot as plt
def define_inital_functions():
    global public_on_ros,pub,pathTopkg,pathToImage
    public_on_ros=False
    
    rospack = rospkg.RosPack()
    pathTopkg=rospack.get_path('ur10_control')
    pathToImage=pathTopkg+"/image.png"
    if(public_on_ros):
        rospy.init_node("camera",anonymous=False)
        pub=rospy.Publisher('/image',sensImg,queue_size=1)
def take_picture():

    bridge=CvBridge()
    webcam=input("Quale webcam vuoi?\n 0)Droidcam\n 1)Webcam\n Risposta:")
    
    cap=cv2.VideoCapture(webcam)
    
    print(cap.isOpened())
    while True:
        ret,frame=cap.read()
        if not ret:
            break
        if(public_on_ros):
            msg=bridge.cv2_to_imgmsg(frame, 'bgr8')
            pub.publish(msg)

        cv2.imshow('frame',frame)
        if(cv2.waitKey(1)==ord('q')):
            break
        if(cv2.waitKey(1)==ord('c')):
            cv2.imwrite(pathToImage,frame)
            print("Imaggine acquisita")
            break
        
    cap.release()
    cv2.destroyAllWindows()
def gradient_on_y(img_to_be_gradient,ksize=5,output=True):
    sobelx64f = cv.Sobel(img_to_be_gradient,cv.CV_64F,0,1,ksize=5)
    abs_sobel64f = np.absolute(sobelx64f)
    sobel_8u = np.uint8(abs_sobel64f)
    if(output):
        plt.subplot(1,2,1),plt.imshow(img_to_be_gradient,cmap = 'gray')
        plt.title('Original'), plt.xticks([]), plt.yticks([])
        #plt.subplot(1,3,2),plt.imshow(sobelx8u,cmap = 'gray')
        #plt.title('Sobel CV_8U'), plt.xticks([]), plt.yticks([])
        plt.subplot(1,2,2),plt.imshow(sobel_8u,cmap = 'gray')
        plt.title('Sobel abs(CV_64F)'), plt.xticks([]), plt.yticks([])
        plt.show()
    return sobel_8u
def smooth_img_kernel(img_to_filter,ksize=7,output=True):

    
    kernel = np.ones((7,7),np.float32)/25
    dst = cv.filter2D(img_to_filter,-1,kernel)
    if(output):
        plt.subplot(121),plt.imshow(img_to_filter,cmap = 'gray'),plt.title('Original')
        plt.xticks([]), plt.yticks([])
        plt.subplot(122),plt.imshow(dst,cmap = 'gray'),plt.title('Averaging')
        plt.xticks([]), plt.yticks([])
        plt.show()
    return dst
def computervision(image,output=True):
    #Da colori a bianco nero
    img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    #Distingui nettamente bianco e nero
    img=filtra_bianco(img)

    cv2.imshow("image",image)
    cv.waitKey(2000)

    cv2.imshow("image",img)
    cv.waitKey(1000)
    
    #Trovo il contorno interessante
    my_cnt=find_my_coutorn(img)
    if(my_cnt==[]):
        print("Errore, nessun cnt troato")
        find_contourns(img,image)
    else:
        cv.drawContours(image, [my_cnt], 0, (0,255,0), 3)
    cv2.imshow("image",image)
    

    #Colora area
    image=colora_area_dentro_countorn(image,my_cnt)
    cv2.imshow("image",image)
    cv2.waitKeyEx()    
    cv2.destroyAllWindows()

    #find_contourns(img,image)
    return img
def colora_area_dentro_countorn(img,cnt):
    dim=img.shape
    for r in range(0,dim[0]):
        for c in range(0,dim[1]):
            isPointInside = cv2.pointPolygonTest(cnt, (c,r),False)
            if(isPointInside>=0):
                img[r,c]=1000
    return img
def find_my_coutorn(img):
    print("Looking for your coutorn")
    ret, thresh = cv.threshold(img, 127, 255, 0)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    my_cnt=[]
    for cnt in contours:
        result=is_the_right_contourn(cnt,img)
        if(result):
            print("Found and exit at first try")
            my_cnt=cnt
            break
    return my_cnt
def find_contourns(img_to_anylize,img_to_draw):
    cont=0
    ret, thresh = cv.threshold(img_to_anylize, 127, 255, 0)
    contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        cont=cont+1
        print(cont)
        cv.drawContours(img_to_draw, [cnt], 0, (0,255,0), 3)  
        cv2.imshow("image",img_to_draw)  
        cv2.waitKeyEx()

    return img_to_draw
def filtro_di_zech(img,output):
    dim=img.shape
    for r in range(1,dim[0]):
        for c in range(1,dim[1]):
            if(img[r,c]>130):
                img[r,c]=1000
            else:
                img[r,c]=0
    
    if(output):
        cv2.imshow('frame',img)
        cv2.waitKeyEx()
    return img    
def visualize_with_cv():

    bridge=CvBridge()
    webcam=input("Quale webcam vuoi?\n 0)Droidcam\n 1)Webcam\n Risposta:")
    
    cap=cv2.VideoCapture(webcam)
    
    print(cap.isOpened())
    while True:
        ret,frame=cap.read()
        if not ret:
            break
        
        image_cv=computervision(frame,output=False)

        
        cv2.imshow('frame',image_cv)
        if(cv2.waitKey(1)==ord('q')):
            break
        
    cap.release()
    cv2.destroyAllWindows()
def filtra_bianco(img):
    dim=img.shape
    for r in range(0,dim[0]):
        for c in range(0,dim[1]):
            if(img[r,c]>220):
                img[r,c]=1000
            else:
                img[r,c]=0
    return img   
def is_the_right_contourn(contourn,img):
    dim=img.shape
    cont_bianchi=0
    cont_neri=0
    for r in range(0,dim[0]):
        for c in range(0,dim[1]):
            isPointInside = cv2.pointPolygonTest(contourn, (c,r),False)
            if(isPointInside>=0):
                if(img[r,c]==232):
                    cont_bianchi=cont_bianchi+1
                elif(img[r,c]==0):
                    cont_neri=cont_neri+1
                else :
                    print(img[r,c])
                    print("Errore")
    
    if(cont_bianchi>300 and cont_neri>1000):
        print(cont_bianchi)
        print(cont_neri)
        print("Potrebbe essere il contourn giusto")
        return True
    return False
def main():
    define_inital_functions()


    value=input("\n 1)Do u wanna take a picture?\n 2)Computer vision\n 3)Visualize webcam with computer vision")
    if(value==1):
        take_picture()
    if(value==2):

        image=cv2.imread(pathToImage)	
        computervision(image,output=True)
    if(value==3):
        visualize_with_cv()

    
if __name__ == '__main__':
    main()


