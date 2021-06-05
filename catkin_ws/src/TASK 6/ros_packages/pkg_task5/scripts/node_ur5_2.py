#!/usr/bin/env python

# importing important packages for the task
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
import json
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
from random import random
import threading
import time
import datetime
import requests
import paho.mqtt.client as mqtt

# empty list to store the positions of red,
# yellow and green packages respectively
pkg_red = []
pkg_yellow = []
pkg_green = []


class Shipped:

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
		self._pub_topic = "/eyrc/vb/AiRiAkAk/ur5_2"
		self._mqtt_client = mqtt.Client()
		self._mqtt_client.on_publish = self.mqtt_on_publish
		self._mqtt_client.connect(self._server_url, self._server_port)
		self._mqtt_client.loop_start()

	def mqtt_on_publish(self, client, userdata, mid):
		rospy.loginfo("Order Placed111111.")

	def shipped_package(self, parameters):
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


# class to implement the task
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

		self._robot_ns = '/' + arg_robot_name
		self._planning_group = "manipulator"
		self._commander = moveit_commander.roscpp_initialize(sys.argv)
		self._robot = moveit_commander.RobotCommander(
			robot_description=self._robot_ns + "/robot_description",
			ns=self._robot_ns)
		self._scene = moveit_commander.PlanningSceneInterface(
			ns=self._robot_ns)
		self._group = moveit_commander.MoveGroupCommander(
			self._planning_group,
			robot_description=self._robot_ns + "/robot_description",
			ns=self._robot_ns)
		self._display_trajectory_publisher = rospy.Publisher(
			self._robot_ns + '/move_group/display_planned_path',
			moveit_msgs.msg.DisplayTrajectory, queue_size=1)
		self._exectute_trajectory_client = actionlib.SimpleActionClient(
			self._robot_ns + '/execute_trajectory',
			moveit_msgs.msg.ExecuteTrajectoryAction)
		self._exectute_trajectory_client.wait_for_server()

		self._planning_frame = self._group.get_planning_frame()
		self._eef_link = self._group.get_end_effector_link()
		self._group_names = self._robot.get_group_names()
		self._box_name = 'box'
		self._curr_state = self._robot.get_current_state()

		rospy.loginfo(
			'\033[94m' + "Planning Group: {}".format(self._planning_frame) +
			'\033[0m')
		rospy.loginfo(
			'\033[94m' + "End Effector Link: {}".format(self._eef_link) +
			'\033[0m')
		rospy.loginfo(
			'\033[94m' + "Group Names: {}".format(self._group_names) +
			'\033[0m')

		rp = rospkg.RosPack()
		self._pkg_path = rp.get_path('pkg_task5')
		self._file_path = self._pkg_path + '/config/saved_trajectories/'
		rospy.loginfo("Package Path: {}".format(self._file_path))

		rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

	def clear_octomap(self):
		clear_octomap_service_proxy = rospy.ServiceProxy(
			self._robot_ns + "/clear_octomap", Empty)
		return clear_octomap_service_proxy()

	# to translate the box using set joint angles
	def set_joint_angles(self, arg_list_joint_angles):
		"""
		Used to move the arm to given set joint angles

		Parameters

		----------

		arg_list_joint_angles : List
																										contains 6 joint angles in a list

		Returns

		-------

		bool
						Path executed or not

		"""

		list_joint_values = self._group.get_current_joint_values()
		self._group.set_joint_value_target(arg_list_joint_angles)
		self._group.plan()
		flag_plan = self._group.go(wait=True)

		list_joint_values = self._group.get_current_joint_values()
		pose_values = self._group.get_current_pose().pose

		if flag_plan is True:
			pass
			# rospy.loginfo(
			#     '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
		else:
			pass
			# rospy.logerr(
			#     '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

		return flag_plan

	def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):
		"""
		Used to move the arm to given set joint angles untill it executes for arg_max_attempts

		Parameters

		----------

		arg_list_joint_angles : List
																										contains 6 joint angles in a list

		arg_max_attempts: Integer
																		   Number of times the planning should be attempted for the given path



		Returns

		-------

		bool
						Path executed or not

		"""

		number_attempts = 0
		flag_success = False

		while ((number_attempts <= arg_max_attempts) and (flag_success is False)):
			number_attempts += 1
			flag_success = self.set_joint_angles(arg_list_joint_angles)
			# rospy.logwarn("attempts: {}".format(number_attempts) )
			# self.clear_octomap()

	# Function to activate/deactivate the vacuum gripper
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

		rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
		try:
			self.gripper = rospy.ServiceProxy(
				'/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
			result = self.gripper(condition)

		# return gripper.result
		except Exception as e:
			rospy.logerr("Service call failed: %s" % e)

	# Function to set_power to conveyor
	def conveyor(self, value):
		"""
		Function to start/ stop the vaccum gripper



		Parameters

		----------

		value : int
														defines the spped of the conveyor

		Returns

		-------

		bool
						Command executed or not
		"""

		rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
		try:
			self.conveyor = rospy.ServiceProxy(
				'/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
			result = self.conveyor(value)

		# return gripper.result
		except Exception as e:
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
		elif condition == 'detach':
			self._scene.remove_attached_object(
				self._eef_link, name=self._box_name)

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
			# rospy.logwarn("attempts: {}".format(number_attempts) )
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

# tfEcho class for taking feed from the logical camera


class tfEcho:

	# Constructor
	def __init__(self):
		"""
		Initialises the transform class to get the coordinates of the packages on conveyor. 

		Parameters

		----------

		Null

		Returns

		-------

		Null

		"""
		# rospy.init_node('node_tf_echo')
		self._tfBuffer = tf2_ros.Buffer()
		self._listener = tf2_ros.TransformListener(self._tfBuffer)

	# Getting coordinates for the red box
	def func_tf1_print(self, arg_frame_1, arg_frame_2):
		"""
		Gets the corrdinates of arg_frame_2 wrt arg_frame_1

		Parameters

		----------

		arg_frame_1 : String
														  Reference Frame
		arg_frame_2 : String
														  Target Frame


		Returns

		-------

		List
						Contains the Coordinates of target frame wrt refernce frame

		"""
		try:
			trans = self._tfBuffer.lookup_transform(
				arg_frame_1, arg_frame_2, rospy.Time())
			L = []
			L.append('True')
			L.append(trans.transform.translation.x)
			L.append(trans.transform.translation.y)
			L.append(trans.transform.translation.z)
			return L

		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			L = ['False']
			rospy.logerr("TF1 error")
			return L


processing = False
new_msg = False
msg = None

# to get data from logical camera


def callback(data):
	global processing, new_msg, msg
	if not processing:
		new_msg = True
		msg = data

# to callback function for fixed number of times


def listener():
	global processing, new_msg, msg
	# rospy.init_node('subscriber')
	rospy.Subscriber('/eyrc/vb/logical_camera_2', LogicalCameraImage, callback)
	r = rospy.Rate(1)
	while not rospy.is_shutdown():
		if new_msg:
			# set processing to True
			processing = True
			new_msg = False
			# simulate a process that take 0.2 seconds
			# rospy.loginfo(len(msg.models))
			# rospy.sleep(0)
			# #set processing to False
			processing = False
			if len(msg.models) >= 1:
				return msg.models
				break


# main function
def get_time_taken(i,o):
	"""
	Calculates the time taken to ship

	Parameters

	----------

	i : Integer
					Number of packeges shipped


	Returns

	-------

	Integer
					Time taken to ship the packages

	"""
	o=float(o)
	v = rospy.get_time()
	print("sim_time",v)
	print("order time",o)
	# print(v)
	if i == 0:
		return v-o
	elif i == 1:
		return v-o
	elif i == 2:
		return v-o
	elif i == 3:
		return v-o
	elif i == 4:
		return v-o
	elif i == 5:
		return v-o
	elif i == 6:
		return v-o
	elif i == 7:
		return v-o
	elif i == 8:
		return v-o


def main(args):
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

	rospy.init_node('node_ur5_2', anonymous=True)

	ur5 = Ur5Moveit(sys.argv[1])
	rospy.sleep(60)

	url2 = "https://spreadsheets.google.com/feeds/list/1xSQAM7_yopXE25Fs" + \
		"blj4me2-frBx9rzoMh8tJAiOgmU/5/public/full?alt=json"
	ssContent2 = requests.get(url2).json()

	while 'entry' not in ssContent2['feed'].keys():
		url2 = "https://spreadsheets.google.com/feeds/list/1xSQAM7_yopXE25Fs" + \
			"blj4me2-frBx9rzoMh8tJAiOgmU/5/public/full?alt=json"
		ssContent2 = requests.get(url2).json()

	box_length = 0.15               # Length of the Package
	vacuum_gripper_width = 0.115    # Vacuum Gripper Width
	delta = vacuum_gripper_width + (box_length/2)  # 0.19

	# creating object of tfEcho to get the coordinates of the packages
	my_tf = tfEcho()
	reference_frame = "world"
	target_frame = "logical_camera_2_"

	# to set the speed of the conveyor
	ur5.conveyor(100)

	# to go to above the conveyor
	ur5.moveit_hard_play_planned_path_from_file(
		ur5._file_path, '0_to_pkg.yaml', 5)
	current = "zero"

	# number of packages shipped
	i = 0

	while True:

		target_frame = "logical_camera_2_"

		# to get the names of models
		x = listener()

		# to check if packages arrives in view of the logical camera
		if len(x) == 1:
			if "packagen" not in str(x[0].type):
				continue
			target_frame += str(x[0].type)+"_frame"
		else:
			if "packagen" not in str(x[1].type):
				continue
			target_frame += str(x[1].type)+"_frame"

		# to get the coordinates of the packages
		val = my_tf.func_tf1_print(reference_frame, target_frame)
		val = my_tf.func_tf1_print(reference_frame, target_frame)
		val = my_tf.func_tf1_print(reference_frame, target_frame)
		val = my_tf.func_tf1_print(reference_frame, target_frame)
		val = my_tf.func_tf1_print(reference_frame, target_frame)

		# to stop the box at desired position
		while val[2] >= 0.05:
			val = my_tf.func_tf1_print(reference_frame, target_frame)

		# to stop the conveyor
		ur5.conveyor(0)

		# move the arm from respective bins to the conveyor
		if i != 0:

			# from red bin
			if current == "redbin":
				ur5.moveit_hard_play_planned_path_from_file(
					ur5._file_path, 'redbin_to_pkg.yaml', 5)

			# from yellow bin
			elif current == "yellowbin":
				ur5.moveit_hard_play_planned_path_from_file(
					ur5._file_path, 'yellowbin_to_pkg.yaml', 5)

			# from green bin
			elif current == "greenbin":
				ur5.moveit_hard_play_planned_path_from_file(
					ur5._file_path, 'greenbin_to_pkg.yaml', 5)

		# for 1st package
		if i == 0:
			ur5.vaccum(True)
		# for rest 8 packages
		else:
			ur5.vaccum(True)

		# attach box to gripper
		ur5.attach_box('attach')

		# to lift the box up
		ur5.hard_set_joint_angles([3.001965295617115, -0.7678599563051787,
								   0.8727562554473742, -
								   1.6756033726890767, -1.5707122165038765, -
								   0.1395453900955168], 5)

		# gets data from incoming order sheet
		url_sub1 = "https://spreadsheets.google.com/feeds"
		url_sub2 = "/list/1xSQAM7_yopXE25Fsblj4m"
		url_sub3 = "e2-frBx9rzoMh8tJAiOgmU/2/public/full?alt=json"
		url = url_sub1+url_sub2+url_sub3
		ssContent = requests.get(url).json()

		# data cleaning
		data_values = ssContent['feed']['entry'][i]['content'].values()
		val = data_values[0].encode("utf-8")
		str_values = list(val.split(","))
		data_separated = []
		for str_value in str_values:
			each_val = str_value.split(":", 1)
			each_val[1] = each_val[1][1:]
			data_separated.append(each_val)
		order_date = datetime.datetime.strptime(
			data_separated[3][1], '%Y-%m-%d %H:%M:%S')

		# gets data from dispatch sheet
		url1 = "https://spreadsheets.google.com/feeds/list/1xSQAM7_yopXE25Fs" + \
			"blj4me2-frBx9rzoMh8tJAiOgmU/3/public/full?alt=json"
		ssContent1 = requests.get(url1).json()

		# data cleaning
		data_values1 = ssContent1['feed']['entry'][i]['content'].values()
		val1 = data_values1[0].encode("utf-8")
		str_values1 = list(val1.split(","))
		data_separated1 = []
		for str_value in str_values1:
			each_val = str_value.split(":", 1)
			each_val[1] = each_val[1][1:]
			data_separated1.append(each_val)


		# gets data from dispatch sheet
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

		print("ur5_2 data")

		print(data_separated2)

		# Moving the ur5 arm along with the attached box to the red bin
		if data_separated[5][1] == "HP":
			ur5.moveit_hard_play_planned_path_from_file(
				ur5._file_path, 'pkg_to_redbin.yaml', 5)
			current = "redbin"
			ur5.conveyor(100)

			# gets time taken
			diff_sec = get_time_taken(i,data_separated2[13][1])
			obj = Shipped()
			parameters = {
				"id": "Dashboard",
				"Team Id": "VB#0566",
				"Unique Id": "AiRiAkAk",
				"Order Id": data_separated[2][1],
				"Item": "Medicine",
				"Priority": "HP",
				"Quantity": 1,
				"City": data_separated[7][1],
				"Longitude": data_separated[8][1],
				"Latitude": data_separated[9][1],
				"Order Dispatched": "YES",
				"Order Shipped": "YES",
				"Order Time": data_separated[3][1],
				"Dispatch Time": data_separated1[9][1],
				"Shipping Time": str(datetime.datetime.now()),
				"Time Taken": str(diff_sec),
				"Cost": 450,
				"Estimated Time of Delivery": "2021-01-23"
			}
			# sending the succes to mqtt
			obj.shipped_package(parameters)

		# Moving the ur5 arm along with the attached box to the yellow bin
		elif data_separated[5][1] == "MP":
			ur5.moveit_hard_play_planned_path_from_file(
				ur5._file_path, 'pkg_to_yellowbin.yaml', 5)
			current = "yellowbin"
			ur5.conveyor(100)

			# gets the time taken
			diff_sec = get_time_taken(i,data_separated2[13][1])
			obj = Shipped()
			parameters = {
				"id": "Dashboard",
				"Team Id": "VB#0566",
				"Unique Id": "AiRiAkAk",
				"Order Id": data_separated[2][1],
				"Item": "Food",
				"Priority": "MP",
				"Quantity": 1,
				"City": data_separated[7][1],
				"Longitude": data_separated[8][1],
				"Latitude": data_separated[9][1],
				"Order Dispatched": "YES",
				"Order Shipped": "YES",
				"Order Time": data_separated[3][1],
				"Dispatch Time": data_separated1[9][1],
				"Shipping Time": str(datetime.datetime.now()),
				"Time Taken": str(diff_sec),
				"Cost": 250,
				"Estimated Time of Delivery": "2021-01-25"
			}
			# sending the success to mqtt
			obj.shipped_package(parameters)

		# Moving the ur5 arm along with the attached box to the green bin
		elif data_separated[5][1] == "LP":
			ur5.moveit_hard_play_planned_path_from_file(
				ur5._file_path, 'pkg_to_greenbin.yaml', 5)
			current = "greenbin"
			ur5.conveyor(100)

			# gets the time taken to ship
			diff_sec = get_time_taken(i,data_separated2[13][1])
			obj = Shipped()
			parameters = {
				"id": "Dashboard", "Team Id": "VB#0566",
				"Unique Id": "AiRiAkAk",
				"Order Id": data_separated[2][1],
				"Item": "Clothes", "Priority": "LP", "Quantity": 1,
				"City": data_separated[7][1],
				"Longitude": data_separated[8][1],
				"Latitude": data_separated[9][1],
				"Order Dispatched": "YES",
				"Order Shipped": "YES",
				"Order Time": data_separated[3][1],
				"Dispatch Time": data_separated1[9][1],
				"Shipping Time": str(datetime.datetime.now()),
				"Time Taken": str(diff_sec), "Cost": 150,
				"Estimated Time of Delivery": "2021-01-27"
			}
			# sending the data to mqtt
			obj.shipped_package(parameters)
		ur5.attach_box('detach')

		# deactivates the gripper
		ur5.vaccum(False)

		i += 1


if __name__ == '__main__':
	main(sys.argv)
