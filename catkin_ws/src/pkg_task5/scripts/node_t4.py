#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import threading
import math
import cv2
import numpy as np
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from pkg_vb_sim.srv import *
from pkg_vb_sim.msg import *
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
from pyzbar import pyzbar
import rospkg
import yaml
import os
import time
#Global variables
l=0
k=0
z=0
x="once"
y=0
v=0
color = 'NA'
package = { "packagen00" : "NA" , "packagen01" : "NA" , "packagen02" : "NA" , "packagen10" : "NA" , "packagen11" : "NA" , "packagen12" : "NA" , "packagen20" : "NA" , "packagen21" : "NA" , "packagen22" : "NA" , "packagen30" : "NA" , "packagen31" : "NA" , "packagen32" : "NA" } 

conveyor_belt = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
con = conveyorBeltPowerMsgRequest()
status = 'NA'


def callback_2(data):
    print("inside callback")
    global v,color,con,conveyor_belt
    v=0
    for i in range(0,len(data.models)):
      if(data.models[i].type != "ur5"): 
       if( data.models[i].pose.position.y <= 0.1 ):
    	   v=1
	   #assign the color of the pacakge detected to color variable
	   if(len(data.models)>1): 
	      color = package[data.models[1].type]


class Camera1:

  def callback(self,data):
    print("image callback")
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)

    (rows,cols,channels) = cv_image.shape
    
    img = cv_image

    height, width = img.shape[0:2]
    
    startRow = int(height*.20)

    startCol = int(width*.05)

    endRow = int(height*.80)
 
    endCol = int(width*.95)

    #cropping the image to get rhe color of all packages
    croppedImage = img[startRow:endRow, startCol:endCol]
    contrast_img = cv2.addWeighted(croppedImage, 2, np.zeros(croppedImage.shape, croppedImage.dtype), 0, 0)
    #cv2.imshow("/eyrc/vb/camera_1/image_raw",contrast_img)
    self.get_qr_data(contrast_img)

    cv2.waitKey(3)
  def __init__(self):
    print("inside init")
    self.bridge = CvBridge()
    #self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", callback_2)
    self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)

  
  def get_qr_data(self, arg_image):
    print("inside get1_qr_data") 
    global x
    qr_result = decode(arg_image)
    print(qr_result)
    print(x)
    #update color once
    if(x == "once"):
	    #Updating the value of x to run below stated code once
	    x = "done"
	    for barcode in pyzbar.decode(arg_image):
	      mydata = barcode.data.decode('utf-8')
	      print(mydata)
	      #to get the top and height of the packages
	      g,h,k,l = barcode.rect

	      #detecting the color of the packages
	      if(g<200):
		 if(h<100):
		    package["packagen00"]= mydata
		 elif(h>100 and h<300):
		    package["packagen10"]= mydata
		 elif(h>300 and h<500):
		    package["packagen20"]= mydata
		 else: 
		   package["packagen30"]= mydata

	      elif(g>200 and g<400):
		 if(h<100):
		    package["packagen01"]= mydata     
		 elif(h>100 and h<300):
		    package["packagen11"]= mydata
		 elif(h>300 and h<500):
		    package["packagen21"]= mydata
		 else:
		    package["packagen31"]= mydata
	       
	      else :
		 if(h<100):
		    package["packagen02"]= mydata
		 elif(h>100 and h<300):
		    package["packagen12"]= mydata
		 elif(h>300 and h<500):
		    package["packagen22"]= mydata
		 else:
		    package["packagen32"]= mydata


	    print(package)
    resized_image = cv2.resize(arg_image, (720/2, 1280/2))
    cv2.imshow("/eyrc/vb/camera_1/image_raw",resized_image)

    if ( len( qr_result ) > 0):
      return (len(qr_result))
    else :
      return ('NA')

  



class Ur5Moveit:

	# Constructor
	def __init__(self, arg_robot_name):

		self._robot_ns = '/'  + arg_robot_name
		self._planning_group = "manipulator"
		
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
		self._box_name = ''


		# Attribute to store computed trajectory by the planner	
		self._computed_plan = ''

		# Current State of the Robot is needed to add box to planning scene
		self._curr_state = self._robot.get_current_state()
		self._group.set_planning_time(60)

		rospy.loginfo(
			'\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
		rospy.loginfo(
			'\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
		rospy.loginfo(
			'\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')


		rp = rospkg.RosPack()
		self._pkg_path = rp.get_path('pkg_task4')
		self._file_path = self._pkg_path + '/config/saved_trajectories/'
		rospy.loginfo( "Package Path: {}".format(self._file_path) )


		rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

	def clear_octomap(self):
		clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
		return clear_octomap_service_proxy()

	def set_joint_angles(self, arg_list_joint_angles):

		list_joint_values = self._group.get_current_joint_values()
		# rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
		# rospy.loginfo(list_joint_values)

		self._group.set_joint_value_target(arg_list_joint_angles)
		self._computed_plan = self._group.plan()
		flag_plan = self._group.go(wait=True)

		list_joint_values = self._group.get_current_joint_values()
		# rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
		# rospy.loginfo(list_joint_values)

		pose_values = self._group.get_current_pose().pose
		# rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
		# rospy.loginfo(pose_values)

		if (flag_plan == True):
			pass
			# rospy.loginfo(
			#     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
		else:
			pass
			# rospy.logerr(
			#     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

		return flag_plan

	def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

		number_attempts = 0
		flag_success = False
		
		while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
			number_attempts += 1
			flag_success = self.set_joint_angles(arg_list_joint_angles)
			rospy.logwarn("attempts: {}".format(number_attempts) )
			# self.clear_octomap()


	def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
		file_path = arg_file_path + arg_file_name
		
		with open(file_path, 'r') as file_open:
			loaded_plan = yaml.load(file_open)
		
		ret = self._group.execute(loaded_plan)
		# rospy.logerr(ret)
		return ret

	
	def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
		number_attempts = 0
		flag_success = False

		while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
			number_attempts += 1
			flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
			rospy.logwarn("attempts: {}".format(number_attempts) )
			# # self.clear_octomap()
		
		return True

	def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
	    box_name = self._box_name
	    scene = self._scene

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
	    ## END_SUB_TUTORIAL


	#ur5_1 planning scene

	def add_box(self, timeout=4):
	    global y
	    box_name = self._box_name
	    scene = self._scene
	    box_pose = geometry_msgs.msg.PoseStamped()
	    box_pose.header.frame_id = "world"
	    #00
	    if(y==0):
	       box_pose.pose.orientation.w = 1.0
	       box_pose.pose.position.x = 0.28
	       box_pose.pose.position.y = -0.41
	       box_pose.pose.position.z = 1.90
	    #01
	    if(y==1):
	       box_pose.pose.orientation.w = 1.0
	       box_pose.pose.position.x = 0.01
	       box_pose.pose.position.y = -0.41
	       box_pose.pose.position.z = 1.90
	    #02
	    if(y==2):
	       box_pose.pose.orientation.w = 1.0
	       box_pose.pose.position.x = -0.29
	       box_pose.pose.position.y = -0.41
	       box_pose.pose.position.z = 1.90
	    #10
	    if(y==10):
	       box_pose.pose.orientation.w = 1.0
	       box_pose.pose.position.x = 0.28
	       box_pose.pose.position.y = -0.41
	       box_pose.pose.position.z = 1.63
	    #11
	    if(y==11):
	       box_pose.pose.orientation.w = 1.0
	       box_pose.pose.position.x = 0.01
	       box_pose.pose.position.y = -0.41
	       box_pose.pose.position.z = 1.63
	    #12
	    if(y==12):
	       box_pose.pose.orientation.w = 1.0
	       box_pose.pose.position.x = -0.29
	       box_pose.pose.position.y = -0.41
	       box_pose.pose.position.z = 1.63
	    #20
	    if(y==20):
	       box_pose.pose.orientation.w = 1.0
	       box_pose.pose.position.x = 0.28
	       box_pose.pose.position.y = -0.41
	       box_pose.pose.position.z = 1.40

	    #21
	    if(y==21):
	       box_pose.pose.orientation.w = 1.0
	       box_pose.pose.position.x = 0.01
	       box_pose.pose.position.y = -0.41
	       box_pose.pose.position.z = 1.40
	    #22
	    if(y==22):
	       box_pose.pose.orientation.w = 1.0
	       box_pose.pose.position.x = -0.27
	       box_pose.pose.position.y = -0.41
	       box_pose.pose.position.z = 1.40
	    #30
	    if(y==30):
	       box_pose.pose.orientation.w = 1.0
	       box_pose.pose.position.x = 0.28
	       box_pose.pose.position.y = -0.41
	       box_pose.pose.position.z = 1.17

	    box_name = "package"
	    scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))
	    self._box_name=box_name

	    return self.wait_for_state_update(box_is_known=True, timeout=timeout)

	# ur5_2 planning scene
	def add_box_2(self, timeout=4):
	    box_name = self._box_name
	    scene = self._scene
	    box_pose = geometry_msgs.msg.PoseStamped()
	    box_pose.header.frame_id = "world"
	    box_pose.pose.orientation.w = 1.0
	    box_pose.pose.position.x = -0.85
	    box_pose.pose.position.y = 0
	    box_pose.pose.position.z = 0.99 
	    box_name = "package$"
	    scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))
	    self._box_name=box_name

	    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


	def attach_box(self, timeout=4):
	    box_name = self._box_name
	    robot = self._robot
	    scene = self._scene
	    eef_link = self._eef_link
	    group_names = self._group_names

	    grasping_group = 'manipulator'
	    touch_links = robot.get_link_names(group=grasping_group)
	    scene.attach_box(eef_link, box_name, touch_links=touch_links)
	    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

	# ur5_2 planning scene
	def attach_box_2(self, timeout=4):
	    box_name = self._box_name
	    robot = self._robot
	    scene = self._scene
	    eef_link = self._eef_link
	    group_names = self._group_names

	    grasping_group = 'manipulator'
	    touch_links = robot.get_link_names(group=grasping_group)
	    scene.attach_box(eef_link, box_name, touch_links=touch_links)
	    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


	def detach_box(self, timeout=4):
	    box_name = self._box_name
	    scene = self._scene
	    eef_link = self._eef_link

	    scene.remove_attached_object(eef_link, name=box_name)

	    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

	def remove_box(self, timeout=4):
	    box_name = self._box_name
	    scene = self._scene
	    scene.remove_world_object(box_name)

	    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)
	    

		
	# Destructor

	def __del__(self):
		moveit_commander.roscpp_shutdown()
		rospy.loginfo(
			'\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

#ur5_1

def ur5_1_thread(name):
    	global y,z,k,l
	ur5 = Ur5Moveit(name)
	vacuum_ur5_1 = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1',vacuumGripper)
	req1 = vacuumGripperRequest()


	home = [math.radians(-180),
			  math.radians(-90),
			  math.radians(0),
			  math.radians(0),
			  math.radians(0),
			  math.radians(0)]

	con.power = 100
	conb = conveyor_belt(con)

	y=00
	ur5.add_box()
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'i_to_h.yaml', 5)
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'h_to_00.yaml', 5)
	req1.activate_vacuum_gripper = True
	resp1 = vacuum_ur5_1(req1)
	ur5.attach_box()
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '00_to_h.yaml', 5)
	req1.activate_vacuum_gripper = False
	resp1 = vacuum_ur5_1(req1)
	ur5.detach_box()
	ur5.remove_box()

	y=01
	ur5.add_box()
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'h_to_01.yaml', 5)
	#rospy.sleep(9)
	rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
	con.power = 100
	conb = conveyor_belt(con)
	req1.activate_vacuum_gripper = True
	resp1 = vacuum_ur5_1(req1)
	ur5.attach_box()
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '01_to_h.yaml', 5)
	req1.activate_vacuum_gripper = False
	resp1 = vacuum_ur5_1(req1)
	k=1
	ur5.detach_box()
	ur5.remove_box()

	y=02
	ur5.add_box()
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'h_to_02.yaml', 5)
	#rospy.sleep(9)
	while(1):
	   if(l==1):
	      break
	rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
	con.power = 100
	conb = conveyor_belt(con)
	req1.activate_vacuum_gripper = True
	resp1 = vacuum_ur5_1(req1)
	ur5.attach_box()
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '02_to_h.yaml', 5)
	req1.activate_vacuum_gripper = False
	resp1 = vacuum_ur5_1(req1)
	k=1
	l=0
	ur5.detach_box()
	ur5.remove_box()

	y=10
	ur5.add_box()
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'h_to_10.yaml', 5)
	#rospy.sleep(9)
	while(1):
	   if(l==1):
	      break
	rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
	con.power = 100
	conb = conveyor_belt(con)

	req1.activate_vacuum_gripper = True
	resp1 = vacuum_ur5_1(req1)
	ur5.attach_box()
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '10_to_h.yaml', 5)
	req1.activate_vacuum_gripper = False
	resp1 = vacuum_ur5_1(req1)
	k=1
	l=0
	ur5.detach_box()
	ur5.remove_box()

	y=11
	ur5.add_box()
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'h_to_11.yaml', 5)
	#rospy.sleep(9)
	while(1):
	   if(l==1):
	      break
	rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
	con.power = 100
	conb = conveyor_belt(con)

	req1.activate_vacuum_gripper = True
	resp1 = vacuum_ur5_1(req1)
	ur5.attach_box()
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '11_to_h.yaml', 5)
	req1.activate_vacuum_gripper = False
	resp1 = vacuum_ur5_1(req1)
	k=1
	l=0
	ur5.detach_box()
	ur5.remove_box()

	y=12
	ur5.add_box()
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'h_to_12.yaml', 5)
	#rospy.sleep(9)
	while(1):
	   if(l==1):
	      break
	rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
	con.power = 100
	conb = conveyor_belt(con)
	req1.activate_vacuum_gripper = True
	resp1 = vacuum_ur5_1(req1)
	ur5.attach_box()
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '12_to_h.yaml', 5)
	req1.activate_vacuum_gripper = False
	resp1 = vacuum_ur5_1(req1)
	k=1
	l=0
	ur5.detach_box()
	ur5.remove_box()

	y=20
	ur5.add_box()
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'h_to_20.yaml', 5)
	#rospy.sleep(9)
	while(1):
	   if(l==1):
	      break
	rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
	con.power = 100
	conb = conveyor_belt(con)

	req1.activate_vacuum_gripper = True
	resp1 = vacuum_ur5_1(req1)
	ur5.attach_box()
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '20_to_h.yaml', 5)
	req1.activate_vacuum_gripper = False
	resp1 = vacuum_ur5_1(req1)
	k=1
	l=0
	ur5.detach_box()
	ur5.remove_box()

	y=21
	ur5.add_box()
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'h_to_21.yaml', 5)
	#rospy.sleep(9)
	while(1):
	   if(l==1):
	      break
	rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
	con.power = 100
	conb = conveyor_belt(con)
	req1.activate_vacuum_gripper = True
	resp1 = vacuum_ur5_1(req1)
	ur5.attach_box()
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '21_to_h.yaml', 5)
	req1.activate_vacuum_gripper = False
	resp1 = vacuum_ur5_1(req1)
	k=1
	l=0
	ur5.detach_box()
	ur5.remove_box()

	y=30
	ur5.add_box()
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'h_to_30.yaml', 5)
	#rospy.sleep(10)
	while(1):
	   if(l==1):
	      break
	rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
	con.power = 100
	conb = conveyor_belt(con)
	req1.activate_vacuum_gripper = True
	resp1 = vacuum_ur5_1(req1)
	ur5.attach_box()
	ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '30_to_h.yaml', 5)
	req1.activate_vacuum_gripper = False
	resp1 = vacuum_ur5_1(req1)
	k=1
	l=0
	ur5.detach_box()
	ur5.remove_box()

	ur5.hard_set_joint_angles(home, 5)
	while(1):
	  if(l==1):
	    k=1
	    break

	rospy.sleep(20)

 
	del ur5

def ur5_2_thread(name):
    global color,v,y,con,conveyor_belt,z,k,l
    ur5_2 = Ur5Moveit(name)
    z=0
    vacuum_ur5_2 = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2',vacuumGripper)
    req2 = vacuumGripperRequest()

    home_angle = [math.radians(7),
			  math.radians(-140),
			  math.radians(-58),
			  math.radians(-79),
			  math.radians(90),
			  math.radians(8)]
    home_angle_1 = [math.radians(7),
			  math.radians(-140),
			  math.radians(-58),
			  math.radians(-70),
			  math.radians(89),
			  math.radians(8)]
    ur5_2_lst_joint_angles_1 = [math.radians(-83),
			  math.radians(-124),
			  math.radians(-76),
			  math.radians(-71),
			  math.radians(89),
			  math.radians(7)]
   

    ur5_2_lst_joint_angles_2 = [math.radians(-159),
			  math.radians(-124),
			  math.radians(-76),
			  math.radians(-71),
			  math.radians(89),
			  math.radians(7)]


    ur5_2_lst_joint_angles_3 = [math.radians(100),
			  math.radians(-124),
			  math.radians(-76),
			  math.radians(-71),
			  math.radians(89),
			  math.radians(7)] 
    ur5_2.hard_set_joint_angles(home_angle_1,5)

    while(1):
      if(v==1):
       rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
       con.power = 0
       conb = conveyor_belt(con)
       v=0
       z=z+1
       rospy.sleep(0.1)
       ur5_2.add_box_2()
       while(1):
	 if(k==1):
	   break
       req2.activate_vacuum_gripper = True
       resp2 = vacuum_ur5_2(req2)
       ur5_2.attach_box_2()
       rospy.sleep(0.1)
 

       if(color == 'red'):
	 ur5_2.hard_set_joint_angles(ur5_2_lst_joint_angles_1,5)  
    
       elif(color == 'yellow'):
	 ur5_2.hard_set_joint_angles(ur5_2_lst_joint_angles_2,5) 

       else:
	 ur5_2.hard_set_joint_angles(ur5_2_lst_joint_angles_3,5) 
       color == 'NA'
       if(z == 8):
	  rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
	  con.power = 100
	  conb = conveyor_belt(con)  
       req2.activate_vacuum_gripper = False
       resp2 = vacuum_ur5_2(req2)
       l=1
       k=0
       ur5_2.detach_box()
       ur5_2.remove_box()
       rospy.sleep(0.1)

       ur5_2.hard_set_joint_angles(home_angle_1,5)
       print(z)

       if(z==9): 
	  break
       
    print('wait')
    del ur5_2

def main():

    rospy.init_node('node_t4', anonymous=True)
    print("inside main")
    #rospy.sleep(10)
    ic = Camera1()
    rospy.Subscriber("/eyrc/vb/logical_camera_2",LogicalCameraImage,callback_2)


    cv2.destroyAllWindows()



if __name__ == '__main__':
	main()
