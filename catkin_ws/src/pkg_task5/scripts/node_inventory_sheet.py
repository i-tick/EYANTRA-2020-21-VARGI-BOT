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
import numpy as np
from random import random
import threading
import time
import requests
import datetime


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
		self._pub_topic = "/eyrc/vb/AiRiAkAk/inventory"

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

		if len(pkg_green) < 3:

			# subsciber used to call the 2D camera
			self.image_sub = rospy.Subscriber(
				"/eyrc/vb/camera_1/image_raw", Image, self.callback)

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
		qr_result = decode(arg_image)
		return qr_result

	# this function is used to implement the image processing
	def callback(self, data):
		# print(data)
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
		def push_google(sku, item, priority, stno, cost, quan):
			
			"""
			Used to push data to google sheet

			"""

			obj = Inventory()
			obj.detected_package(sku, item, priority, stno, cost, quan)
			

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


		for code in codes:
			rospy.sleep(2)
			x, y, w, h = code.rect.left, code.rect.top, code.rect.width, code.rect.height
			cv2.rectangle(resized_image, (x, y), (x+w, y+h), (255, 0, 0), 8)

			# for red package
			if code.data == "red":

				if code.rect.left in range(60, 68):

					if code.rect.top in range(155, 159):
						pkg_red.append('00')
						push_google("R00", "Medicine", "HP", "R0 C0", 450, 1)

					elif code.rect.top in range(245, 249):
						pkg_red.append('10')
						push_google("R10", "Medicine", "HP", "R1 C0", 450, 1)

					elif code.rect.top in range(319, 325):
						pkg_red.append('20')
						push_google("R20", "Medicine", "HP", "R2 C0", 450, 1)

					elif code.rect.top in range(395, 399):
						pkg_red.append('30')
						push_google("R30", "Medicine", "HP", "R3 C0", 450, 1)

				elif code.rect.left in range(155, 159):

					if code.rect.top in range(155, 159):
						pkg_red.append('01')
						push_google("R01", "Medicine", "HP", "R0 C1", 450, 1)

					elif code.rect.top in range(245, 249):
						pkg_red.append('11')
						push_google("R11", "Medicine", "HP", "R1 C1", 450, 1)

					elif code.rect.top in range(319, 325):
						pkg_red.append('21')
						push_google("R21", "Medicine", "HP", "R2 C1", 450, 1)

					elif code.rect.top in range(395, 399):
						pkg_red.append('31')
						push_google("R31", "Medicine", "HP", "R3 C1", 450, 1)

				elif code.rect.left in range(249, 255):

					if code.rect.top in range(155, 159):
						pkg_red.append('02')
						push_google("R02", "Medicine", "HP", "R0 C2", 450, 1)

					elif code.rect.top in range(245, 249):
						pkg_red.append('12')
						push_google("R12", "Medicine", "HP", "R1 C2", 450, 1)

					elif code.rect.top in range(319, 325):
						pkg_red.append('22')
						push_google("R22", "Medicine", "HP", "R2 C2", 450, 1)

					elif code.rect.top in range(395, 399):
						pkg_red.append('32')
						push_google("R32", "Medicine", "HP", "R3 C2", 450, 1)

			# for green package
			elif code.data == "green":

				if code.rect.left in range(60, 68):

					if code.rect.top in range(155, 159):
						pkg_green.append('00')
						push_google("G00", "Clothes", "LP", "R0 C0", 150, 1)

					elif code.rect.top in range(245, 249):
						pkg_green.append('10')
						push_google("G10", "Clothes", "LP", "R1 C0", 150, 1)

					elif code.rect.top in range(319, 325):
						pkg_green.append('20')
						push_google("G20", "Clothes", "LP", "R2 C0", 150, 1)

					elif code.rect.top in range(395, 399):
						pkg_green.append('30')
						push_google("G30", "Clothes", "LP", "R3 C0", 150, 1)

				elif code.rect.left in range(155, 160):

					if code.rect.top in range(155, 160):
						pkg_green.append('01')
						push_google("G01", "Clothes", "LP", "R0 C1", 150, 1)

					elif code.rect.top in range(245, 249):
						pkg_green.append('11')
						push_google("G11", "Clothes", "LP", "R1 C1", 150, 1)

					elif code.rect.top in range(319, 325):
						pkg_green.append('21')
						push_google("G21", "Clothes", "LP", "R2 C1", 150, 1)

					elif code.rect.top in range(395, 399):
						pkg_green.append('31')
						push_google("G31", "Clothes", "LP", "R3 C1", 150, 1)

				elif code.rect.left in range(249, 255):

					if code.rect.top in range(155, 159):
						pkg_green.append('02')
						push_google("G02", "Clothes", "LP", "R0 C2", 150, 1)

					elif code.rect.top in range(245, 249):
						pkg_green.append('12')
						push_google("G12", "Clothes", "LP", "R1 C2", 150, 1)

					elif code.rect.top in range(319, 325):
						pkg_green.append('22')
						push_google("G22", "Clothes", "LP", "R2 C2", 150, 1)

					elif code.rect.top in range(395, 399):
						pkg_green.append('32')
						push_google("G32", "Clothes", "LP", "R3 C2", 150, 1)

			# for yellow package
			elif code.data == "yellow":

				if code.rect.left in range(60, 68):

					if code.rect.top in range(155, 159):
						pkg_yellow.append('00')
						push_google("Y00", "Food", "MP", "R0 C0", 250, 1)

					elif code.rect.top in range(245, 249):
						pkg_yellow.append('10')
						push_google("Y10", "Food", "MP", "R1 C0", 250, 1)

					elif code.rect.top in range(319, 325):
						pkg_yellow.append('20')
						push_google("Y20", "Food", "MP", "R2 C0", 250, 1)

					elif code.rect.top in range(395, 399):
						pkg_yellow.append('30')
						push_google("Y30", "Food", "MP", "R3 C0", 250, 1)

				elif code.rect.left in range(155, 161):

					if code.rect.top in range(155, 159):
						pkg_yellow.append('01')
						push_google("Y01", "Food", "MP", "R0 C1", 250, 1)

					elif code.rect.top in range(245, 249):
						pkg_yellow.append('11')
						push_google("Y11", "Food", "MP", "R1 C1", 250, 1)

					elif code.rect.top in range(319, 325):
						pkg_yellow.append('21')
						push_google("Y21", "Food", "MP", "R2 C1", 250, 1)
					elif code.rect.top in range(390, 400):
						pkg_yellow.append('31')
						push_google("Y31", "Food", "MP", "R3 C1", 250, 1)

				elif code.rect.left in range(249, 255):

					if code.rect.top in range(155, 159):
						pkg_yellow.append('02')
						push_google("Y02", "Food", "MP", "R0 C2", 250, 1)

					elif code.rect.top in range(245, 249):
						pkg_yellow.append('12')
						push_google("Y12", "Food", "MP", "R1 C2", 250, 1)

					elif code.rect.top in range(319, 325):
						pkg_yellow.append('22')
						push_google("Y22", "Food", "MP", "R2 C2", 250, 1)

					elif code.rect.top in range(395, 399):
						pkg_yellow.append('32')
						push_google("Y32", "Food", "MP", "R3 C2", 250, 1)


	  
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
					sku = "R"+i
					stno = "R"+i[0]+" C"+i[1]
					# print(sku)
					# print(stno)
					push_google(sku, "Medicine", "HP", stno, 450, 1)
				if package[i] == "yellow" and i not in pkg_yellow:
					pkg_yellow.append(i)
					sku = "Y"+i
					stno = "R"+i[0]+" C"+i[1]
					# print(sku)
					# print(stno)
					push_google(sku, "Food", "MP", stno, 250, 1)
				if package[i] == "green" and i not in pkg_green:
					pkg_green.append(i)
					sku = "G"+i
					stno = "R"+i[0]+" C"+i[1]
					# print(sku)
					# print(stno)
					push_google(sku, "Clothes", "LP", stno, 150, 1)
		
		
		print(pkg_red)
		print(pkg_yellow)
		print(pkg_green)
			

		cv2.waitKey(10)


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
