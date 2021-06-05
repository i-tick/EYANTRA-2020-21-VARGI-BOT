#!/usr/bin/env python

import rospy
import cv2
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from pkg_vb_sim.msg import LogicalCameraImage
from cv_bridge import CvBridge, CvBridgeError

from pyzbar.pyzbar import decode
import rospkg

import yaml
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math

import tf2_ros
import tf2_msgs.msg
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg
from std_srvs.srv import Empty

import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from pyzbar.pyzbar import decode

from random import random
import threading
import time
import requests


pkg_red=[]
pkg_yellow=[]
pkg_green=[]
sheet=[]
c=0
class Ur5Moveit:

    # Constructor
    def __init__(self, arg_robot_name):

        #rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        self._exectute_trajectory_client = actionlib.SimpleActionClient( self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = 'box'
        #self._sheet_url=param_config_iot['google_apps']['spread_sheet_id']

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        # rospy.loginfo(
        #     '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        # rospy.loginfo(
        #     '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        # rospy.loginfo(
        #     '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')
        
        # rp = rospkg.RosPack()
        # self._pkg_path = rp.get_path('pkg_moveit_examples')
        # self._file_path = self._pkg_path + '/config/saved_trajectories/'
        # rospy.loginfo( "Package Path: {}".format(self._file_path) )

        # rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def get_qr_data(self, arg_image):
        qr_result = decode(arg_image)
        color=[]
        for i in qr_result:
            color.append(i.data)
        return qr_result
        if ( len( qr_result ) > 0):
            return (qr_result[0].data)
        else :
            return ('NA')



    def callback(self,data):
        def push_google(sku,item,priority,stno,cost,quan):
            parameters = {"id":"Inventory","Team Id":"VB#0566","Unique Id":"AiRiAkAk","SKU":sku,"Item":item,"Priority":priority,"Storage Number":stno,"Cost":cost,"Quantity":quan}
            sheet.append(parameters)
            #print(sheet)
            #URL = "https://script.google.com/macros/s/AKfycby_M9efwsF0Bc5DQc4X7CpIWALNYtvC6oyzcVUqiTSUaVdm7Y92KluR/exec"
            #response = requests.get(URL, params=parameters)
            #print(response.content)

        global c
        if c!=0:
            return
        c=1
        global pkg_red,pkg_yellow,pkg_green
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)


        ####################################

        (rows,cols,channels) = cv_image.shape
        image = cv_image
        height, width = img.shape[0:2]
        startRow = int(height*.20)
        startCol = int(width*.05)
        endRow = int(height*.80)
        endCol = int(width*.95)
        #cropping the image to get rhe color of all packages
        croppedImage = img[startRow:endRow, startCol:endCol]
        contrast_img = cv2.addWeighted(croppedImage, 2, np.zeros(croppedImage.shape, croppedImage.dtype), 0, 0)
        #cv2.imshow("/eyrc/vb/camera_1/image_raw",contrast_img)
        codes = self.get_qr_data(contrast_img)


        ####################################
        # (rows,cols,channels) = cv_image.shape
        # image = cv_image
        # resized_image = cv2.resize(image, (720/2, 1280/2)) 
        # gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
        # ret,thresh3 = cv2.threshold(gray,41,255,cv2.THRESH_BINARY)
        # codes = self.get_qr_data(thresh3)
        color = []
        for code in codes:
            color.append(code.data)
            x, y, w, h = code.rect.left, code.rect.top, code.rect.width, code.rect.height
            cv2.rectangle(resized_image, (x,y),(x+w, y+h),(255, 0, 0), 8)
            # print(code)
            # print("---------------------")
            if code.data=="red":
                if code.rect.left in range(60,68):
                    if code.rect.top in range(155,159):
                        pkg_red.append('00')
                        #push_google("R000121","Medicine","HP","R0 C0",450,1)
                        # parameters = {"id":"Inventory","Team Id":"VB#0566","Unique Id":"AiRiAkAk","SKU":"R000121","Item":"Medicine","Priority":"HP","Storage Number":"R0 C0","Cost":450,"Quantity":1}
                        # sheet.append(parameters)
                        # #print(sheet)
                        # URL = "https://script.google.com/macros/s/AKfycby_M9efwsF0Bc5DQc4X7CpIWALNYtvC6oyzcVUqiTSUaVdm7Y92KluR/exec"
                        # response = requests.get(URL, params=parameters)
                    elif code.rect.top in range(245,249):
                        pkg_red.append('10')
                        #push_google("R100121","Medicine","HP","R1 C0",450,1)
                        # parameters = {"id":"Inventory","Team Id":"VB#0566","Unique Id":"AiRiAkAk","SKU":"R000121","Item":"Medicine","Priority":"HP","Storage Number":"R0 C0","Cost":450,"Quantity":1}
                        # sheet.append(parameters)
                        # #print(sheet)
                        # URL = "https://script.google.com/macros/s/AKfycby_M9efwsF0Bc5DQc4X7CpIWALNYtvC6oyzcVUqiTSUaVdm7Y92KluR/exec"
                        # response = requests.get(URL, params=parameters)
                    elif code.rect.top in range(319,325):
                        pkg_red.append('20')
                        #push_google("R200121","Medicine","HP","R2 C0",450,1)
                        # parameters = {"id":"Inventory","Team Id":"VB#0566","Unique Id":"AiRiAkAk","SKU":"R000121","Item":"Medicine","Priority":"HP","Storage Number":"R0 C0","Cost":450,"Quantity":1}
                        # sheet.append(parameters)
                        # #print(sheet)
                        # URL = "https://script.google.com/macros/s/AKfycby_M9efwsF0Bc5DQc4X7CpIWALNYtvC6oyzcVUqiTSUaVdm7Y92KluR/exec"
                        # response = requests.get(URL, params=parameters)
                                #elif code.rect.top in range(395,399):
                                #	pkg_red.append('30')
                elif code.rect.left in range(155,159):
                    if code.rect.top in range(155,159):
                        pkg_red.append('01')
                        #push_google("R010121","Medicine","HP","R0 C1",450,1)
                        # parameters = {"id":"Inventory","Team Id":"VB#0566","Unique Id":"AiRiAkAk","SKU":"R000121","Item":"Medicine","Priority":"HP","Storage Number":"R0 C0","Cost":450,"Quantity":1}
                        # sheet.append(parameters)
                        # #print(sheet)
                        # URL = "https://script.google.com/macros/s/AKfycby_M9efwsF0Bc5DQc4X7CpIWALNYtvC6oyzcVUqiTSUaVdm7Y92KluR/exec"
                        # response = requests.get(URL, params=parameters)
                    elif code.rect.top in range(245,249):
                        pkg_red.append('11')
                        #push_google("R110121","Medicine","HP","R1 C1",450,1)
                    elif code.rect.top in range(319,325):
                        pkg_red.append('21')
                        #push_google("R210121","Medicine","HP","R2 C1",450,1)
                        # parameters = {"id":"Inventory","Team Id":"VB#0566","Unique Id":"AiRiAkAk","SKU":"R210121","Item":"Medicine","Priority":"HP","Storage Number":"R2 C1","Cost":450,"Quantity":1}
                        # sheet.append(parameters)
                        # #print(sheet)
                        # URL = "https://script.google.com/macros/s/AKfycby_M9efwsF0Bc5DQc4X7CpIWALNYtvC6oyzcVUqiTSUaVdm7Y92KluR/exec"
                        # response = requests.get(URL, params=parameters)
                                #elif code.rect.top in range(395,399):
                                #	pkg_red.append('31')
                elif code.rect.left in range(249,255):
                    if code.rect.top in range(155,159):
                        pkg_red.append('02')
                        #push_google("R020121","Medicine","HP","R0 C2",450,1)
                    elif code.rect.top in range(245,249):
                        pkg_red.append('12')
                        #push_google("R120121","Medicine","HP","R1 C2",450,1)
                        # parameters = {"id":"Inventory","Team Id":"VB#0566","Unique Id":"AiRiAkAk","SKU":"R120121","Item":"Medicine","Priority":"HP","Storage Number":"R1 C2","Cost":450,"Quantity":1}
                        # sheet.append(parameters)
                        # #print(sheet)
                        # URL = "https://script.google.com/macros/s/AKfycby_M9efwsF0Bc5DQc4X7CpIWALNYtvC6oyzcVUqiTSUaVdm7Y92KluR/exec"
                        # response = requests.get(URL, params=parameters)
                    elif code.rect.top in range(319,325):
                        pkg_red.append('22')
                        #push_google("R220121","Medicine","HP","R2 C2",450,1)
                                # elif code.rect.top in range(395,399):
                                # 	pkg_red.append('32')
            elif code.data=="green":
                if code.rect.left in range(60,68):
                    if code.rect.top in range(155,159):
                        pkg_green.append('00')
                        #push_google("G000121","Clothes","LP","R0 C0",150,1)
                    elif code.rect.top in range(245,249):
                        pkg_green.append('10')
                        #push_google("G100121","Clothes","LP","R1 C0",150,1)
                    elif code.rect.top in range(319,325):
                        pkg_green.append('20')
                        #push_google("G200121","Clothes","LP","R2 C0",150,1)
                            # elif code.rect.top in range(395,399):
                            # 	pkg_green.append('30')
                elif code.rect.left in range(155,159):
                    if code.rect.top in range(155,159):
                        pkg_green.append('01')
                        #push_google("G010121","Clothes","LP","R0 C1",150,1)
                    elif code.rect.top in range(245,249):
                        pkg_green.append('11')
                        #push_google("G110121","Clothes","LP","R1 C1",150,1)
                    elif code.rect.top in range(319,325):
                        pkg_green.append('21')
                        #push_google("G210121","Clothes","LP","R2 C1",150,1)
                                # elif code.rect.top in range(395,399):
                                # 	pkg_green.append('31')
                elif code.rect.left in range(249,255):
                    if code.rect.top in range(155,159):
                        pkg_green.append('02')
                        #push_google("G020121","Clothes","LP","R0 C2",150,1)
                    elif code.rect.top in range(245,249):
                        pkg_green.append('12')
                        #push_google("G120121","Clothes","LP","R1 C2",150,1)
                    elif code.rect.top in range(319,325):
                        pkg_green.append('22')
                        #push_google("G220121","Clothes","LP","R2 C2",150,1)
                                # elif code.rect.top in range(395,399):
                                # 	pkg_green.append('32')
            elif code.data=="yellow":
                if code.rect.left in range(60,68):
                    if code.rect.top in range(155,159):
                        pkg_yellow.append('00')
                        #push_google("Y000121","Food","MP","R0 C0",250,1)
                    elif code.rect.top in range(245,249):
                        pkg_yellow.append('10')
                        #push_google("Y100121","Food","MP","R1 C0",250,1)
                    elif code.rect.top in range(319,325):
                        pkg_yellow.append('20')
                        #push_google("Y200121","Food","MP","R2 C0",250,1)
                                # elif code.rect.top in range(395,399):
                                # 	pkg_yellow.append('30')
                elif code.rect.left in range(155,161):
                    if code.rect.top in range(155,159):
                        pkg_yellow.append('01')
                        #push_google("Y010121","Food","MP","R0 C1",250,1)
                    elif code.rect.top in range(245,249):
                        pkg_yellow.append('11')
                        #push_google("Y110121","Food","MP","R1 C1",250,1)
                    elif code.rect.top in range(319,325):
                        pkg_yellow.append('21')
                        #push_google("Y210121","Food","MP","R2 C1",250,1)
                                # elif code.rect.top in range(390,400):
                                # 	pkg_yellow.append('31')
                elif code.rect.left in range(249,255):
                    if code.rect.top in range(155,159):
                        pkg_yellow.append('02')
                        #push_google("Y020121","Food","MP","R0 C2",250,1)
                    elif code.rect.top in range(245,249):
                        pkg_yellow.append('12')
                        #push_google("Y120121","Food","MP","R1 C2",250,1)
                    elif code.rect.top in range(319,325):
                        pkg_yellow.append('22')
                        #push_google("Y220121","Food","MP","R2 C2",250,1)
                                # elif code.rect.top in range(395,399):
                                # 	pkg_yellow.append('32')


            cv2.rectangle(resized_image, code.polygon[0], code.polygon[1],(0, 255, 0), 4)
            cv2.putText(resized_image, code.data+str(code.rect.top), (x-1, y-1),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        text1 = 'No. Codes: %s' % len(codes)
        cv2.putText(resized_image, text1, (5, 15),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        # cv2.imshow('bounding box', im)
        cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)
        print(pkg_yellow)
        print(pkg_red)
        print(pkg_green)
        print()
        # (rows,cols,channels) = cv_image.shape
        
        # image = cv_image

        # # Resize a 720x1280 image to 360x640 to fit it on the screen
        # resized_image = cv2.resize(image, (720/2, 1280/2)) 

        # cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)
        
        # rospy.loginfo(self.get_qr_data(image))


        return
        
        cv2.waitKey(3)










def main():
  #pkg_red=["00","12","21"]
  #pkg_yellow=["02","11","20"]
  #pkg_green = ["01","10","22"]


  
  rospy.init_node('test_5', anonymous=True)
  ur5 = Ur5Moveit(sys.argv[1])


if __name__ == '__main__':
    main()