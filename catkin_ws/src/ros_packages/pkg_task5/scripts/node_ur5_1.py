#! /usr/bin/env python

# Importing required packages

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from sensor_msgs.msg import Image
from pkg_vb_sim.msg import LogicalCameraImage
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode
import actionlib
import paho.mqtt.client as mqtt
import json
import rospkg
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import conveyorBeltPowerMsg
import yaml
import pyzbar
import numpy as np
import os
import math
import time
import sys
import copy
import json
import requests
import cv2
from std_srvs.srv import Empty
import datetime

# Empty lists to store the positions of red , yellow and green packages respectively
pkg_red = []
pkg_yellow = []
pkg_green = []

c = 0
x="once"
color = 'NA'
package = { "00" : "NA" , "01" : "NA" , "02" : "NA" , "10" : "NA" , "11" : "NA" , "12" : "NA" , "20" : "NA" , "21" : "NA" , "22" : "NA" , "30" : "NA" , "31" : "NA" , "32" : "NA" } 

status = 'NA'


class Dispatch:

	def __init__(self):
		"""
		Initializes the server url , servel port , pub topic of the MQTT

		Connecting the client to the server

		Parameters

		----------

		Null

		Returns

		-------

		Null

		"""

		self._server_url = "broker.mqttdashboard.com"
		self._server_port = 1883
		self._qos = 0
		self._pub_topic = "/eyrc/vb/AiRiAkAk/ur5_1"
		self._mqtt_client = mqtt.Client()
		self._mqtt_client.on_publish = self.mqtt_on_publish
		self._mqtt_client.connect(self._server_url, self._server_port)
		self._mqtt_client.loop_start()

	def mqtt_on_publish(self, client, userdata, mid):
		# print("mid: " + str(mid))
		rospy.loginfo("Package Dispatch.")

	def dispatch_package(self, parameters):
		"""
		Sends the message to the MQTT whenever the order is dispatched

		Parameters

		----------

		parameters : Dictionary
														 Contains the data (Unique ID, Color, Cost, Priority, etc.) of the dispatched packages

		Returns

		-------

		Null

		"""

		str_payload_2 = json.dumps(parameters)
		(rc, mid) = self._mqtt_client.publish(
			self._pub_topic, str_payload_2, qos=self._qos)

# Class to operate the ur5 1 arm


class Ur5Moveit:
	# Constructor
	def __init__(self, arg_robot_name):
		"""
		Initializes the arm one

		Parameters

		----------

		arg_robot_name : String
																		  Name of the ur5 arm 1

		Returns

		-------

		Null

		"""
		self.bridge = CvBridge()

		if len(pkg_green) < 3:

			# subsciber used to call the 2D camera
			self.image_sub = rospy.Subscriber(
				"/eyrc/vb/camera_1/image_raw", Image, self.callback)

		self._robot_ns = '/' + arg_robot_name
		self._planning_group = "manipulator"

		self._commander = moveit_commander.roscpp_initialize(sys.argv)
		self._robot = moveit_commander.RobotCommander(
			robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
		self._scene = moveit_commander.PlanningSceneInterface(
			ns=self._robot_ns)
		self._group = moveit_commander.MoveGroupCommander(
			self._planning_group, robot_description=self._robot_ns + "/robot_description", ns=self._robot_ns)
		self._display_trajectory_publisher = rospy.Publisher(
			self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
		self._exectute_trajectory_client = actionlib.SimpleActionClient(
			self._robot_ns + '/execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
		self._exectute_trajectory_client.wait_for_server()

		self._planning_frame = self._group.get_planning_frame()
		self._eef_link = self._group.get_end_effector_link()
		self._group_names = self._robot.get_group_names()
		self._box_name = ''

		# Attribute to store computed trajectory by the planner
		self._computed_plan = ''

		# Current State of the Robot is needed to add box to planning scene
		self._curr_state = self._robot.get_current_state()

		rospy.loginfo(
			'\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
		rospy.loginfo(
			'\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
		rospy.loginfo(
			'\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

		rp = rospkg.RosPack()
		self._pkg_path = rp.get_path('pkg_task5')
		self._file_path = self._pkg_path + '/config/saved_trajectories/'
		rospy.loginfo("Package Path: {}".format(self._file_path))

		rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

	# this is used to extract the info from the qr code detected such as color, position, etc.
	def get_qr_data(self, arg_image):
		"""
		Used to extract the info from the qr code detected such as color , position ,etc

		Parameters

		----------

		arg_image : List
														Contains the raw data of the image

		Returns

		-------

		List
						Extracted data from the image
		String
						Whenever no result is obtained

		"""
		qr_result = decode(arg_image)
		color = []
		for i in qr_result:
			color.append(i.data)
		return qr_result
		if (len(qr_result) > 0):
			return (qr_result[0].data)
		else:
			return ('NA')

	# this function is used to implement the image processing
	def callback(self, data):
		"""
		Used to implement Image Processing


		Perform necessary image processing tasks and identify the position of each detected package on the shelf

		Parameters

		---------

		data : String
										/eyrc/vb/camera_1/image_raw

		Returns

		-------

		Null

		"""

		global c,x
		if c != 0:
			return
		c = 1

		global pkg_red, pkg_yellow, pkg_green
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.logerr(e)

		(rows, cols, channels) = cv_image.shape

		image = cv_image

		resized_image = cv2.resize(image, (720/2, 1280/2))
		gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
		ret, thresh3 = cv2.threshold(gray, 41, 255, cv2.THRESH_BINARY)

		codes = self.get_qr_data(thresh3)
		color = []

		for code in codes:
			color.append(code.data)
			x, y, w, h = code.rect.left, code.rect.top, code.rect.width, code.rect.height
			cv2.rectangle(resized_image, (x, y), (x+w, y+h), (255, 0, 0), 8)

			# for red package
			if code.data == "red":

				if code.rect.left in range(60, 68):

					if code.rect.top in range(155, 159):
						pkg_red.append('00')

					elif code.rect.top in range(245, 249):
						pkg_red.append('10')

					elif code.rect.top in range(319, 325):
						pkg_red.append('20')

					elif code.rect.top in range(395, 399):
						pkg_red.append('30')

				elif code.rect.left in range(155, 159):

					if code.rect.top in range(155, 159):
						pkg_red.append('01')

					elif code.rect.top in range(245, 249):
						pkg_red.append('11')

					elif code.rect.top in range(319, 325):
						pkg_red.append('21')

					elif code.rect.top in range(395, 399):
						pkg_red.append('31')

				elif code.rect.left in range(249, 255):

					if code.rect.top in range(155, 159):
						pkg_red.append('02')

					elif code.rect.top in range(245, 249):
						pkg_red.append('12')

					elif code.rect.top in range(319, 325):
						pkg_red.append('22')

					elif code.rect.top in range(395, 399):
						pkg_red.append('32')

			# for green package
			elif code.data == "green":

				if code.rect.left in range(60, 68):

					if code.rect.top in range(155, 159):
						pkg_green.append('00')

					elif code.rect.top in range(245, 249):
						pkg_green.append('10')

					elif code.rect.top in range(319, 325):
						pkg_green.append('20')

					elif code.rect.top in range(395, 399):
						pkg_green.append('30')

				elif code.rect.left in range(155, 159):

					if code.rect.top in range(155, 159):
						pkg_green.append('01')

					elif code.rect.top in range(245, 249):
						pkg_green.append('11')

					elif code.rect.top in range(319, 325):
						pkg_green.append('21')

					elif code.rect.top in range(395, 399):
						pkg_green.append('31')

				elif code.rect.left in range(249, 255):

					if code.rect.top in range(155, 159):
						pkg_green.append('02')

					elif code.rect.top in range(245, 249):
						pkg_green.append('12')

					elif code.rect.top in range(319, 325):
						pkg_green.append('22')

					elif code.rect.top in range(395, 399):
						pkg_green.append('32')

			# for yellow package
			elif code.data == "yellow":

				if code.rect.left in range(60, 68):

					if code.rect.top in range(155, 159):
						pkg_yellow.append('00')

					elif code.rect.top in range(245, 249):
						pkg_yellow.append('10')

					elif code.rect.top in range(319, 325):
						pkg_yellow.append('20')

					elif code.rect.top in range(395, 399):
						pkg_yellow.append('30')

				elif code.rect.left in range(155, 161):

					if code.rect.top in range(155, 159):
						pkg_yellow.append('01')

					elif code.rect.top in range(245, 249):
						pkg_yellow.append('11')

					elif code.rect.top in range(319, 325):
						pkg_yellow.append('21')
					elif code.rect.top in range(390, 400):
						pkg_yellow.append('31')

				elif code.rect.left in range(249, 255):

					if code.rect.top in range(155, 159):
						pkg_yellow.append('02')

					elif code.rect.top in range(245, 249):
						pkg_yellow.append('12')

					elif code.rect.top in range(319, 325):
						pkg_yellow.append('22')

					elif code.rect.top in range(395, 399):
						pkg_yellow.append('32')

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
		contrast_img = cv2.addWeighted(croppedImage, 2, np.zeros(croppedImage.shape, croppedImage.dtype), 1, 0)
		


		#print(qr_result)
		#update color once
		if(True):
			#Updating the value of x to run below stated code once
			
			for barcode in pyzbar.pyzbar.decode(contrast_img):
				mydata = barcode.data.decode('utf-8')
			
				#to get the top and height of the packages
				g,h,k,l = barcode.rect
				# print(mydata)

				#detecting the color of the packages
				if(g<200):
					if(h<100):
						package["00"]= mydata
					elif(h>100 and h<300):
						package["10"]= mydata
					elif(h>300 and h<500):
						package["20"]= mydata
					else:
						package["30"]= mydata
				elif(g>200 and g<400):
					if(h<100):
						package["01"]= mydata     
					elif(h>100 and h<300):
						package["11"]= mydata
					elif(h>300 and h<500):
						package["21"]= mydata
					else:
						package["31"]= mydata
		   
				else :
					if(h<100):
						package["02"]= mydata
					elif(h>100 and h<300):
						package["12"]= mydata
					elif(h>300 and h<500):
						package["22"]= mydata
					else:
						package["32"]= mydata

			
			for i in package.keys():
				if package[i] == "red" and i not in pkg_red:
					pkg_red.append(i)
					
				if package[i] == "yellow" and i not in pkg_yellow:
					pkg_yellow.append(i)
					
				if package[i] == "green" and i not in pkg_green:
					pkg_green.append(i)
		
		cv2.waitKey(1)

	def clear_octomap(self):
		clear_octomap_service_proxy = rospy.ServiceProxy(
			self._robot_ns + "/clear_octomap", Empty)
		return clear_octomap_service_proxy()

	# to execute the arm using the saved path in config folder
	def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
		"""
		Used to play the saved trajectory file present in the config folder

		The ur5 1 arm executes the required motion with the help of the saved trajectory files present in the config folder

		Parameters

		----------

		arg_file_path : String
																		Location of the saved file
		arg_file_name : String
																		Name of the saved file

		Returns

		-------

		bool
						Path executed or not

		"""
		file_path = arg_file_path + arg_file_name

		with open(file_path, 'r') as file_open:
			loaded_plan = yaml.load(file_open)

		ret = self._group.execute(loaded_plan)
		# rospy.logerr(ret)
		return ret

	def vaccum(self, condition):
		"""
		Function to activate / deactive the vaccum gripper



		Parameters

		----------

		condition : bool
														Activates / deactives the vacuum gripper based on it's value

		Returns

		-------

		bool
						Command executed or not
		"""

		rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1')
		try:
			self.gripper = rospy.ServiceProxy(
				'/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
			result = self.gripper(condition)
			return result

		# return gripper.result
		except Exception as e:
			# self.vaccum(condition)
			rospy.logerr("Service call failed: %s" % e)

	# Function to attach the box to the vacuum gripper
	def attach_box(self, condition):
		"""
		Function to attach the box to the vacuum gripper

		Parameters

		----------

		condition : String
														Whether the box is to be attached or detached

		Returns

		-------

		NULL

		"""

		self._scene = moveit_commander.PlanningSceneInterface(
			ns=self._robot_ns)
		grasping_group = 'manipulator'
		touch_links = self._robot.get_link_names(group=grasping_group)
		if condition == 'attach':
			self._scene.attach_box(
				self._eef_link, self._box_name, touch_links=touch_links)
			rospy.loginfo("ATTCHED")
		elif condition == 'detach':
			self._scene.remove_attached_object(
				self._eef_link, name=self._box_name)
			rospy.loginfo("DEATCHED")

	# to execute the arm using saved path for given number of times if failed

	def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
		"""
		Used to play the saved trajectory file present in the config folder

		The ur5 1 arm executes the required motion with the help of the saved trajectory files present in the config folder till arg_max_attempts time

		Parameters

		----------

		arg_file_path : String
																		Location of the saved file
		arg_file_name : String
																		Name of the saved file
		arg_max_attempts : Integer
																						Number of times the planning should be attempted for the given path

		Returns

		-------

		bool
						True

		"""
		number_attempts = 0
		flag_success = False

		while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
			number_attempts += 1
			flag_success = self.moveit_play_planned_path_from_file(
				arg_file_path, arg_file_name)
			rospy.logwarn("attempts: {}".format(number_attempts))
			# # self.clear_octomap()

		return True

	# Destructor

	def __del__(self):
		"""
		To delete the class

		Parameters

		----------

		NULL

		Returns

		-------

		NULL

		"""
		moveit_commander.roscpp_shutdown()
		rospy.loginfo(
			'\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():
	"""
	Main function of the program

	It contains all the function calls to execute the given task that includes all the necessary logic , function deeclarations , etc.

	Parameters

	----------
	NULL

	Returns

	--------

	NULL

	"""
	rospy.init_node('node_ur5_1', anonymous=True)
	rospy.sleep(15)
	ur5 = Ur5Moveit(sys.argv[1])
	#rospy.sleep(40)

	while not rospy.is_shutdown():

		# get data from incoming orders
		url = "https://spreadsheets.google.com/feeds/list/1xSQAM7_yopXE25Fsblj4me2-frBx9rzoMh8tJAiOgmU/2/public/full?alt=json"
		ssContent1 = requests.get(url).json()

		url2 = "https://spreadsheets.google.com/feeds/list/1xSQAM7_yopXE25Fs" + \
				"blj4me2-frBx9rzoMh8tJAiOgmU/5/public/full?alt=json"
		ssContent2 = requests.get(url2).json()

		# Storing the list of positions of red boxes in a set
		red_set = set(pkg_red)

		# Storing the list of positions of yellow boxes in a set
		yellow_set = set(pkg_yellow)

		# Storing the list of positions of green boxes in a set
		green_set = set(pkg_green)

		# Storing the positions of first three red boxes in a list
		red_list = list(red_set)
		red_list.sort()
		if len(red_list) >= 3:
			red_list = red_list[:3]

		# Storing the positions of first three yellow boxes in a list
		yellow_list = list(yellow_set)
		yellow_list.sort()
		if len(yellow_list) >= 3:
			yellow_list = yellow_list[:3]

		# Storing the positions of first three green boxes in a list
		green_list = list(green_set)
		green_list.sort()
		if len(green_list) >= 3:
			green_list = green_list[:3]

		print(red_list)
		print(green_list)
		print(yellow_list)
		# till sheet is empty
		while 'entry' not in ssContent1['feed'].keys():
			url = "https://spreadsheets.google.com/feeds/list/1xSQAM7_yopXE25Fsblj4me2-frBx9rzoMh8tJAiOgmU/2/public/full?alt=json"
			ssContent1 = requests.get(url).json()
		while 'entry' not in ssContent2['feed'].keys():
			url2 = "https://spreadsheets.google.com/feeds/list/1xSQAM7_yopXE25Fs" + \
				"blj4me2-frBx9rzoMh8tJAiOgmU/5/public/full?alt=json"
			ssContent2 = requests.get(url2).json()

		# number of packages dispatched
		i = 0
		while (i < 9):

			while (len(ssContent1['feed']['entry']) < (i+1)):
				url = "https://spreadsheets.google.com/feeds/list/1xSQAM7_yopXE25Fsblj4me2-frBx9rzoMh8tJAiOgmU/2/public/full?alt=json"
				ssContent1 = requests.get(url).json()


			# data cleaing
			data_values = ssContent1['feed']['entry'][i]['content'].values()
			val = data_values[0].encode("utf-8")
			str_values = list(val.split(","))
			# print(str_values)
			data_separated = []
			for str_value in str_values:
				each_val = str_value.split(":", 1)
				each_val[1] = each_val[1][1:]
				data_separated.append(each_val)

			url2 = "https://spreadsheets.google.com/feeds/list/1xSQAM7_yopXE25Fs" + \
				"blj4me2-frBx9rzoMh8tJAiOgmU/5/public/full?alt=json"
			ssContent2 = requests.get(url2).json()

			while (len(ssContent2['feed']['entry']) < (i+1)):
				url2 = "https://spreadsheets.google.com/feeds/list/1xSQAM7_yopXE25Fs" + \
					"blj4me2-frBx9rzoMh8tJAiOgmU/5/public/full?alt=json"
				ssContent2 = requests.get(url2).json()

			# data cleaning
			data_values2 = ssContent2['feed']['entry'][i]['content'].values()
			val2 = data_values2[0].encode("utf-8")
			str_values2 = list(val2.split(","))
			data_separated2 = []
			for str_value in str_values2:
				each_val = str_value.split(":", 1)
				each_val[1] = each_val[1][1:]
				data_separated2.append(each_val)

			print("data2")
			print(data_separated2)

			initial_box_path = "zero_to_"
			box_path = "conveyor_to_"
			conveyor_path = "_to_conveyor.yaml"

			# picks red packages
			if data_separated[5][1] == "HP":

				if (i == 0):
					# if red is in 1st row
					path1 = initial_box_path + red_list[0]+".yaml"
					path2 = red_list[0] + conveyor_path
					del red_list[0]

					# plays path from saved trajectory
					ur5.moveit_hard_play_planned_path_from_file(
						ur5._file_path, path1, 5)
					ur5.vaccum(True)
					ur5.attach_box('attach')
					ur5.moveit_hard_play_planned_path_from_file(
						ur5._file_path, path2, 5)
					ur5.vaccum(False)
					ur5.attach_box('detach')
					obj = Dispatch()

					parameters = {"id": "OrdersDispatched", "Team Id": "VB#0566", "Unique Id": "AiRiAkAk", "Order ID": data_separated[2][1], "City": data_separated[7][
						1], "Item": "Medicine", "Priority": "HP", "Cost": data_separated[10][1], "Dispatch Status": "YES", "Dispatch Date and Time": datetime.datetime.now()}
					date_dt3 = datetime.datetime.strptime(
						data_separated[3][1], '%Y-%m-%d %H:%M:%S')
					parameters_1 = {"id": "Dashboard", "Team Id": "VB#0566", "Unique Id": "AiRiAkAk", "Order Id": data_separated[2][1], "Item": "Medicine", "Priority": "HP", "Quantity": 1, "City": data_separated[7][1], "Longitude": data_separated[
						8][1], "Latitude": data_separated[9][1], "Order Dispatched": "YES", "Order Shipped": "NO", "Order Time": str(date_dt3), "Dispatch Time": str(datetime.datetime.now()), "Shipping Time": "", "Time Taken": "", "Cost": 450,"Incoming Sim Time":data_separated2[11][1]}

					# sends data to mqtt
					obj.dispatch_package(parameters_1)

				else:

					# if red box is not in 1st row
					path1 = box_path + red_list[0]+".yaml"
					path2 = red_list[0] + conveyor_path
					del red_list[0]

					# plays path from saved trajectory
					ur5.moveit_hard_play_planned_path_from_file(
						ur5._file_path, path1, 5)
					ur5.vaccum(True)
					ur5.attach_box('attach')
					ur5.moveit_hard_play_planned_path_from_file(
						ur5._file_path, path2, 5)
					ur5.vaccum(False)
					ur5.attach_box('detach')

					parameters = {"id": "OrdersDispatched", "Team Id": "VB#0566", "Unique Id": "AiRiAkAk", "Order ID": data_separated[2][1], "City": data_separated[7][
						1], "Item": "Medicine", "Priority": "HP", "Dispatch Quantity": "1", "Cost": data_separated[10][1], "Dispatch Status": "YES", "Dispatch Date and Time": str(datetime.datetime.now())}
					date_dt3 = str(datetime.datetime.strptime(
						data_separated[3][1], '%Y-%m-%d %H:%M:%S'))
					parameters_1 = {"id": "Dashboard", "Team Id": "VB#0566", "Unique Id": "AiRiAkAk", "Order Id": data_separated[2][1], "Item": "Medicine", "Priority": "HP", "Quantity": 1, "City": data_separated[7][
						1], "Longitude": data_separated[8][1], "Latitude": data_separated[9][1], "Order Dispatched": "YES", "Order Shipped": "NO", "Order Time": date_dt3, "Dispatch Time": str(datetime.datetime.now()), "Cost": 450,"Incoming Sim Time":data_separated2[11][1]}
					obj = Dispatch()

					# sends data to mqtt
					obj.dispatch_package(parameters_1)

			# picks yelow packages
			elif data_separated[5][1] == "MP":

				if i == 0:
					# if package in 1st row
					path1 = initial_box_path + yellow_list[0]+".yaml"
					path2 = yellow_list[0] + conveyor_path
					del yellow_list[0]

					# plays path from saved trajectory
					ur5.moveit_hard_play_planned_path_from_file(
						ur5._file_path, path1, 5)
					ur5.vaccum(True)
					ur5.attach_box('attach')
					ur5.moveit_hard_play_planned_path_from_file(
						ur5._file_path, path2, 5)
					ur5.vaccum(False)
					ur5.attach_box('detach')

					parameters = {"id": "OrdersDispatched", "Team Id": "VB#0566", "Unique Id": "AiRiAkAk", "Order ID": data_separated[2][1], "City": data_separated[7][
						1], "Item": "Food", "Priority": "MP", "Dispatch Quantity": "1", "Cost": data_separated[10][1], "Dispatch Status": "YES", "Dispatch Date and Time": str(datetime.datetime.now())}
					date_dt3 = str(datetime.datetime.strptime(
						data_separated[3][1], '%Y-%m-%d %H:%M:%S'))
					parameters_1 = {"id": "Dashboard", "Team Id": "VB#0566", "Unique Id": "AiRiAkAk", "Order Id": data_separated[2][1], "Item": "Food", "Priority": "MP", "Quantity": 1, "City": data_separated[7][
						1], "Longitude": data_separated[8][1], "Latitude": data_separated[9][1], "Order Dispatched": "YES", "Order Shipped": "NO", "Order Time": date_dt3, "Dispatch Time": str(datetime.datetime.now()), "Cost": 250,"Incoming Sim Time":data_separated2[11][1]}
					obj = Dispatch()

					# sends data to mqtt
					obj.dispatch_package(parameters_1)

				else:
					# if package is not in st row
					path1 = box_path + yellow_list[0]+".yaml"
					path2 = yellow_list[0] + conveyor_path
					del yellow_list[0]
					ur5.moveit_hard_play_planned_path_from_file(
						ur5._file_path, path1, 5)
					ur5.vaccum(True)
					ur5.attach_box('attach')
					ur5.moveit_hard_play_planned_path_from_file(
						ur5._file_path, path2, 5)

					ur5.vaccum(False)
					ur5.attach_box('detach')
					parameters = {"id": "OrdersDispatched", "Team Id": "VB#0566", "Unique Id": "AiRiAkAk", "Order ID": data_separated[2][1], "City": data_separated[7][
						1], "Item": "Food", "Priority": "MP", "Dispatch Quantity": "1", "Cost": data_separated[10][1], "Dispatch Status": "YES", "Dispatch Date and Time": str(datetime.datetime.now())}
					date_dt3 = str(datetime.datetime.strptime(
						data_separated[3][1], '%Y-%m-%d %H:%M:%S'))
					parameters_1 = {"id": "Dashboard", "Team Id": "VB#0566", "Unique Id": "AiRiAkAk", "Order Id": data_separated[2][1], "Item": "Food", "Priority": "MP", "Quantity": 1, "City": data_separated[7][
						1], "Longitude": data_separated[8][1], "Latitude": data_separated[9][1], "Order Dispatched": "YES", "Order Shipped": "NO", "Order Time": date_dt3, "Dispatch Time": str(datetime.datetime.now()), "Cost": 250,"Incoming Sim Time":data_separated2[11][1]}
					obj = Dispatch()

					# send data to mqtt
					obj.dispatch_package(parameters_1)

			# picks yellow package
			elif data_separated[5][1] == "LP":
				if i == 0:
					# if package in st row
					path1 = initial_box_path + green_list[0]+".yaml"
					path2 = green_list[0] + conveyor_path
					del green_list[0]
					ur5.moveit_hard_play_planned_path_from_file(
						ur5._file_path, path1, 5)
					ur5.vaccum(True)
					ur5.attach_box('attach')
					ur5.moveit_hard_play_planned_path_from_file(
						ur5._file_path, path2, 5)
					ur5.vaccum(False)
					ur5.attach_box('detach')
					parameters = {"id": "OrdersDispatched", "Team Id": "VB#0566", "Unique Id": "AiRiAkAk", "Order ID": data_separated[2][1], "City": data_separated[7][
						1], "Item": "Clothes", "Priority": "LP", "Dispatch Quantity": "1", "Cost": data_separated[10][1], "Dispatch Status": "YES", "Dispatch Date and Time": str(datetime.datetime.now())}
					date_dt3 = str(datetime.datetime.strptime(
						data_separated[3][1], '%Y-%m-%d %H:%M:%S'))
					parameters_1 = {"id": "Dashboard", "Team Id": "VB#0566", "Unique Id": "AiRiAkAk", "Order Id": data_separated[2][1], "Item": "Clothes", "Priority": "LP", "Quantity": 1, "City": data_separated[7][
						1], "Longitude": data_separated[8][1], "Latitude": data_separated[9][1], "Order Dispatched": "YES", "Order Shipped": "NO", "Order Time": date_dt3, "Dispatch Time": str(datetime.datetime.now()), "Cost": 150,"Incoming Sim Time":data_separated2[11][1]}
					URL = "https://script.google.com/macros/s/AKfycbyaL3hI8IpTETKV6ZG7xNtn3_yhY9SXzIkAFCbjtjLJnprVH6ol_T0C/exec"
					obj = Dispatch()
					# send data to mqtt
					obj.dispatch_package(parameters_1)

				else:

					# if package is not in 1st row
					path1 = box_path + green_list[0]+".yaml"
					path2 = green_list[0] + conveyor_path
					del green_list[0]
					ur5.moveit_hard_play_planned_path_from_file(
						ur5._file_path, path1, 5)
					ur5.vaccum(True)
					ur5.attach_box('attach')
					ur5.moveit_hard_play_planned_path_from_file(
						ur5._file_path, path2, 5)
					ur5.vaccum(False)
					ur5.attach_box('detach')
					parameters = {"id": "OrdersDispatched", "Team Id": "VB#0566", "Unique Id": "AiRiAkAk", "Order ID": data_separated[2][1], "City": data_separated[7][
						1], "Item": "Clothes", "Priority": "LP", "Dispatch Quantity": "1", "Cost": data_separated[10][1], "Dispatch Status": "YES", "Dispatch Date and Time": str(datetime.datetime.now())}
					date_dt3 = str(datetime.datetime.strptime(
						data_separated[3][1], '%Y-%m-%d %H:%M:%S'))
					parameters_1 = {"id": "Dashboard", "Team Id": "VB#0566", "Unique Id": "AiRiAkAk", "Order Id": data_separated[2][1], "Item": "Clothes", "Priority": "LP", "Quantity": 1, "City": data_separated[7][
						1], "Longitude": data_separated[8][1], "Latitude": data_separated[9][1], "Order Dispatched": "YES", "Order Shipped": "NO", "Order Time": date_dt3, "Dispatch Time": str(datetime.datetime.now()), "Cost": 150,"Incoming Sim Time":data_separated2[11][1]}
					obj = Dispatch()
					# sends data to mqtt
					obj.dispatch_package(parameters_1)

			i += 1
		break

	del ur5


if __name__ == '__main__':
	main()
