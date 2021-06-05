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

status = 'NA'



class Camera1:

	def __init__(self):
		print("init")
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback)
		print("init_end")

  
	def get_qr_data(self, arg_image): 
		print("get_qr_data")
		global x
		qr_result = decode(arg_image)
		#print(qr_result)
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

  
	def callback(self,data):
    		print("callback")
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
		self.get_qr_data(contrast_img)

		cv2.waitKey(3)

def main():
	print("main")
	rospy.init_node('node_t4', anonymous=True)
	ic = Camera1()
	# rospy.Subscriber("/eyrc/vb/logical_camera_2",LogicalCameraImage,callback_2)


#     cv2.destroyAllWindows()



if __name__ == '__main__':
	main()
