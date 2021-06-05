#! /usr/bin/env python

# importing the required package
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import paho.mqtt.client as mqtt
import json
import math
import rospkg
import yaml
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
import pyzbar
from random import random
import threading
import time
import requests
import datetime
import numpy as np


# empty list to store the positions of red, yellow and green packages respectively
pkg_red = []
pkg_yellow = []
pkg_green = []
c = 0
x="once"
color = 'NA'
package = { "00" : "NA" , "01" : "NA" , "02" : "NA" , "10" : "NA" , "11" : "NA" , "12" : "NA" , "20" : "NA" , "21" : "NA" , "22" : "NA" , "30" : "NA" , "31" : "NA" , "32" : "NA" } 

status = 'NA'



class Inventory:

	def __init__(self):
		"""
		Initialzes the Inventory Class

		Parameters

		----------

		NULL

		Returns

		-------

		NULL

		"""

		self._server_url = "broker.mqttdashboard.com"
		self._server_port = 1883
		self._qos = 0
		self._pub_topic = "/eyrc/vb/AitikDan/inventory"

		self._mqtt_client = mqtt.Client()
		self._mqtt_client.on_publish = self.mqtt_on_publish
		self._mqtt_client.connect(self._server_url, self._server_port)
		self._mqtt_client.loop_start()

	def mqtt_on_publish(self, client, userdata, mid):
		"""
		Publishes Package Detected on the terminal
		"""
		rospy.loginfo("Package Detected.")

	def detected_package(self, sku, item, priority, stno, cost, quan):
		"""
		Function Used to define parameters such as id , SKU , item , cos etc. and convet them ino json

		Parameters

		----------

		sku : String
										  Contains the SKU of the package
		item : String
										Description of the item i.e. food , clothes or medicines
		priority : String
																										High , Medium or Low Priority
		cost : Int
																		Cost of the package

		quan : Int
																		Quantity

		Returns

		-------

		NULL


		"""
		d = datetime.datetime.now()
		mon = d.strftime("%m")
		year = d.strftime("%y")
		mon += year
		sku += mon
		parameters = {
			"id": "Inventory",
			"Team Id": "VB#0566",
			"Unique Id": "AiRiAkAk",
			"SKU": sku,
			"Item": item,
			"Priority": priority,
			"Storage Number": stno,
			"Cost": cost,
			"Quantity": quan
		}
		str_payload = json.dumps(parameters)

		(rc, mid) = self._mqtt_client.publish(
			self._pub_topic, str_payload, qos=self._qos)

# class to implement the task


class Ur5Moveit:

	# Constructor
	def __init__(self):
		"""
		Initializes the Ur5Moveit Class and subscribes to the 2D Camera

		Parameters

		----------

		NULL

		Returns

		-------

		NULL

		"""
		self.bridge = CvBridge()

		self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image, self.callback)

	# this is used to extract the info from the qr code detected such as color, position, etc.
	def get_qr_data(self, arg_image):
		"""
		Function to extract information from the qr code setected such as color , position ,etc

		Parameters 

		----------

		arg_image : List
																										Contains the raw data of the image

		Returns

		-------

		List : 
																		Extracted data from image


		"""
		# print("get_qr_data")

		def push_google(sku, item, priority, stno, cost, quan):
			"""
            Used to push data to google sheet

            """

			obj = Inventory()
			obj.detected_package(sku, item, priority, stno, cost, quan)
			d = datetime.datetime.now()

			mon = d.strftime("%m")
			year = d.strftime("%y")
			mon += year
			sku += mon
			parameters = {"id": "Inventory", "Team Id": "VB#0566", "Unique Id": "AiRiAkAk", "SKU": sku,
                          "Item": item, "Priority": priority, "Storage Number": stno, "Cost": cost, "Quantity": quan}

			URL = "https://script.google.com/macros/s/AKfycby_M9efwsF0Bc5DQc4X7CpIWALNYtvC6oyzcVUqiTSUaVdm7Y92KluR/exec"

		global x
		qr_result = decode(arg_image)
		#print(qr_result)
		#update color once
		if(x == "once"):
			#Updating the value of x to run below stated code once
			x = "done"
			for barcode in pyzbar.pyzbar.decode(arg_image):
				mydata = barcode.data.decode('utf-8')
				# print(mydata)
				#to get the top and height of the packages
				g,h,k,l = barcode.rect

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
				if package[i] == "red":
					print(package[i])
					pkg_red.append(i)
					sku = "R"+i
					stno = "R"+i[0]+" C"+i[1]
					# print(sku)
					print(stno)
					push_google(sku, "Medicine", "HP", stno, 450, 1)
				if package[i] == "yellow":
					print(package[i])
					pkg_yellow.append(i)
					sku = "Y"+i
					stno = "R"+i[0]+" C"+i[1]
					# print(sku)
					print(stno)
					push_google(sku, "Food", "MP", stno, 250, 1)
				if package[i] == "green":
					print(package[i])
					pkg_green.append(i)
					sku = "G"+i
					stno = "R"+i[0]+" C"+i[1]
					# print(sku)
					print(stno)
					push_google(sku, "Clothes", "LP", stno, 150, 1)

		resized_image = cv2.resize(arg_image, (720/2, 1280/2))
		cv2.imshow("/eyrc/vb/camera_1/image_raw",resized_image)

		if ( len( qr_result ) > 0):
			return (len(qr_result))
		else :
			return ('NA')


	# this function is used to implement the image processing
	def callback(self, data):

		# print(data)
		# print("callback")
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
		contrast_img = cv2.addWeighted(croppedImage, 1, np.zeros(croppedImage.shape, croppedImage.dtype), 1, 0)
		self.get_qr_data(contrast_img)


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
	rospy.sleep(10)
	ur5 = Ur5Moveit()
	rospy.sleep(71)


if __name__ == '__main__':
	main()
