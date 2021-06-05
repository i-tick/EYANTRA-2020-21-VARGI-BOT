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
import requests
import datetime


# empty list to store the positions of red, yellow and green packages respectively
pkg_red=[]
pkg_yellow=[]
pkg_green=[]

# class to implement the task
class Ur5Moveit:

    # Constructor
    def __init__(self, arg_robot_name):

        self._robot_ns = '/'  + arg_robot_name
        self._planning_group = "manipulator"
        self.bridge = CvBridge()
        if len(pkg_green)<3:

            # subsciber used to call the 2D camera 
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

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        
        rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_moveit_examples')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    # this is used to extract the info from the qr code detected such as color, position, etc.
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

    # this function is used to implement the image processing
    def callback(self,data):
        global pkg_red,pkg_yellow,pkg_green
        try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        (rows,cols,channels) = cv_image.shape
        image = cv_image
        resized_image = cv2.resize(image, (720/2, 1280/2)) 
        gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
        ret,thresh3 = cv2.threshold(gray,41,255,cv2.THRESH_BINARY)
        codes = self.get_qr_data(thresh3)
        color = []
        for code in codes:
            color.append(code.data)
            x, y, w, h = code.rect.left, code.rect.top, code.rect.width, code.rect.height
            cv2.rectangle(resized_image, (x,y),(x+w, y+h),(255, 0, 0), 8)
            
            # for red package
            if code.data=="red":

                if code.rect.left in range(60,68):
                
                    if code.rect.top in range(155,159):
                        pkg_red.append('00')
                
                    elif code.rect.top in range(245,249):
                        pkg_red.append('10')
                
                    elif code.rect.top in range(319,325):
                        pkg_red.append('20')
					
                    elif code.rect.top in range(395,399):
						pkg_red.append('30')

                elif code.rect.left in range(155,159):
                    
                    if code.rect.top in range(155,159):
                        pkg_red.append('01')
                    
                    elif code.rect.top in range(245,249):
                        pkg_red.append('11')
                    
                    elif code.rect.top in range(319,325):
                        pkg_red.append('21')
					
                    elif code.rect.top in range(395,399):
						pkg_red.append('31')

                elif code.rect.left in range(249,255):
                
                    if code.rect.top in range(155,159):
                        pkg_red.append('02')
                
                    elif code.rect.top in range(245,249):
                        pkg_red.append('12')
                
                    elif code.rect.top in range(319,325):
                        pkg_red.append('22')
                    elif code.rect.top in range(395,399):
                        pkg_red.append('32')
            
            # for green package
            elif code.data=="green":
                
                if code.rect.left in range(60,68):
                
                    if code.rect.top in range(155,159):
                        pkg_green.append('00')
                
                    elif code.rect.top in range(245,249):
                        pkg_green.append('10')
                
                    elif code.rect.top in range(319,325):
                        pkg_green.append('20')
                    
                    elif code.rect.top in range(395,399):
                        pkg_green.append('30')
                
                elif code.rect.left in range(155,159):
                
                    if code.rect.top in range(155,159):
                        pkg_green.append('01')
                
                    elif code.rect.top in range(245,249):
                        pkg_green.append('11')
                
                    elif code.rect.top in range(319,325):
                        pkg_green.append('21')
                    
                    elif code.rect.top in range(395,399):
						pkg_green.append('31')
                
                elif code.rect.left in range(249,255):
                
                    if code.rect.top in range(155,159):
                        pkg_green.append('02')
                
                    elif code.rect.top in range(245,249):
                        pkg_green.append('12')
                
                    elif code.rect.top in range(319,325):
                        pkg_green.append('22')
                    elif code.rect.top in range(395,399):
						pkg_green.append('32')
            # for yellow package
            elif code.data=="yellow":
            
                if code.rect.left in range(60,68):
            
                    if code.rect.top in range(155,159):
                        pkg_yellow.append('00')
            
                    elif code.rect.top in range(245,249):
                        pkg_yellow.append('10')
            
                    elif code.rect.top in range(319,325):
                        pkg_yellow.append('20')
                    elif code.rect.top in range(395,399):
						pkg_yellow.append('30')
            
                elif code.rect.left in range(155,161):
            
                    if code.rect.top in range(155,159):
                        pkg_yellow.append('01')
                    
                    elif code.rect.top in range(245,249):
                        pkg_yellow.append('11')
                    
                    elif code.rect.top in range(319,325):
                        pkg_yellow.append('21')
                    
                    elif code.rect.top in range(390,400):
						pkg_yellow.append('31')
            
                elif code.rect.left in range(249,255):
            
                    if code.rect.top in range(155,159):
                        pkg_yellow.append('02')
            
                    elif code.rect.top in range(245,249):
                        pkg_yellow.append('12')
            
                    elif code.rect.top in range(319,325):
                        pkg_yellow.append('22')
                    elif code.rect.top in range(395,399):
						pkg_yellow.append('32')

		
			# to draw the polydon around the boxes and write the color of the packages
            cv2.rectangle(resized_image, code.polygon[0], code.polygon[1],(0, 255, 0), 4)
            cv2.putText(resized_image, code.data+str(code.rect.top), (x-1, y-1),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        text1 = 'No. Codes: %s' % len(codes)
        cv2.putText(resized_image, text1, (5, 15),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.waitKey(1)


    def clear_octomap(self):
		clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
		return clear_octomap_service_proxy()

    # to translate the box using set joint angles
    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        # rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        # rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
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

    # to translate the box using set joint angles with given number of attempts if failed
    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            # rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()

    # Function to activate/deactivate the vacuum gripper
    def vaccum(self,condition):

        rospy.wait_for_service('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2')
        try:
            self.gripper = rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
            result=self.gripper(condition)
            return result

        #return gripper.result
        except Exception as e:
            rospy.sleep(1)
            self.vaccum(condition)
            rospy.logerr("Service call failed: %s"%e)

    #Function to set_power to conveyor
    def conveyor(self,value):

        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        try:
            self.conveyor = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
            result=self.conveyor(value)
            return result

        #return gripper.result
        except Exception as e:
            rospy.sleep(1)
            self.conveyor(value)
            rospy.logerr("Service call failed: %s"%e)

    #Function to attach the box to the vacuum gripper
    def attach_box(self,condition):

        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        grasping_group = 'manipulator'
        touch_links = self._robot.get_link_names(group=grasping_group)
        if condition == 'attach':
            self._scene.attach_box(self._eef_link, self._box_name, touch_links=touch_links)
        elif condition == 'detach':
            self._scene.remove_attached_object(self._eef_link, name=self._box_name)


    #Function to remove box from the planning scene
    def remove_box(self, timeout=4):

        box_name = self._box_name
        scene = self._scene
        scene.remove_world_object(box_name)
    	
    # to execute the arm using the saved path in config folder
    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
		file_path = arg_file_path + arg_file_name
		
		with open(file_path, 'r') as file_open:
			loaded_plan = yaml.load(file_open)
		
		ret = self._group.execute(loaded_plan)
		# rospy.logerr(ret)
		return ret

    # to execute the arm using saved path for given number of times if failed
    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
		number_attempts = 0
		flag_success = False

		while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
			number_attempts += 1
			flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
			# rospy.logwarn("attempts: {}".format(number_attempts) )
			# # self.clear_octomap()
		return True


    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

# tfEcho class for taking feed from the logical camera
class tfEcho:
    
    #Constructor
    def __init__(self):
        #rospy.init_node('node_tf_echo')
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)

    #Getting coordinates for the red box
    def func_tf1_print(self, arg_frame_1, arg_frame_2):
        try:
            trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time())
	    L=[]
	    L.append('True')
	    L.append(trans.transform.translation.x)
	    L.append(trans.transform.translation.y)
	    L.append(trans.transform.translation.z)
	    return L

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
	    L=['False']
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
            #set processing to True
            processing = True
            new_msg = False
            #simulate a process that take 0.2 seconds
            # rospy.loginfo(len(msg.models))
            # rospy.sleep(0)
            # #set processing to False
            processing = False
            if len(msg.models)>=1:
              return msg.models
              break


# main function

def main(args):

  rospy.init_node('node_ur5_2', anonymous=True)
  
  ur5 = Ur5Moveit(sys.argv[1])
#   rospy.sleep(5)
  

  #Storing the list of positions of red boxes in a set
  red_set=set(pkg_red)

  #Storing the list of positions of yellow boxes in a set
  yellow_set=set(pkg_yellow)

  #Storing the list of positions of green boxes in a set
  green_set=set(pkg_green)



  #Storing the positions of first three red boxes in a list
  red_list=list(red_set)
  red_list.sort()
  if len(red_list)>=3:
    red_list=red_list[:3]

  #Storing the positions of first three yellow boxes in a list
  yellow_list=list(yellow_set)
  yellow_list.append('01')
  yellow_list.sort()
  print(yellow_list)
  if len(yellow_list)>=3:
    yellow_list=yellow_list[:3]


  #Storing the positions of first three green boxes in a list
  green_list=list(green_set)
  green_list.sort()
  if len(green_list)>=3:
    green_list=green_list[:3]

    
  box_length = 0.15               # Length of the Package
  vacuum_gripper_width = 0.115    # Vacuum Gripper Width
  delta = vacuum_gripper_width + (box_length/2)  # 0.19    

  current = "zero"
  # to go to above the conveyor
  ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, '0_to_pkg.yaml', 5)
  current = "conveyor"
  # creating object of tfEcho to get the coordinates of the packages
  my_tf = tfEcho()
  reference_frame = "world"
  target_frame = "logical_camera_2_"
  
  # to set the speed of the conveyor
  ur5.conveyor(100)

  i=0
  while True:

    target_frame = "logical_camera_2_"
    
    # to get the names of models 
    x = listener()
   
    if len(x)==1:
        if "packagen" not in str(x[0].type):
            continue
        target_frame +=str(x[0].type)+"_frame"
    else:
        if "packagen" not in str(x[1].type):
            continue
        target_frame +=str(x[1].type)+"_frame"
    
    # to print the models(package) found 
    print(target_frame)
    
    # to store the coordinates of the package
    val=my_tf.func_tf1_print(reference_frame, target_frame)
    val=my_tf.func_tf1_print(reference_frame, target_frame)
    val=my_tf.func_tf1_print(reference_frame, target_frame)
    val=my_tf.func_tf1_print(reference_frame, target_frame)
    val=my_tf.func_tf1_print(reference_frame, target_frame)
    val=my_tf.func_tf1_print(reference_frame, target_frame)

    # to stop the box at desired position
    while val[2]>=0.05:
        val = my_tf.func_tf1_print(reference_frame, target_frame)
    
    # to stop the conveyor
    ur5.conveyor(0)

    

    # to move the arm to box on the conveyor

    print(current)
    # to red box
    if current == "redbin":
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'redbin_to_pkg.yaml', 5)
    # to yellow box
    elif current == "yellowbin":
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'yellowbin_to_pkg.yaml', 5)
    # to green box
    elif current == "greenbin":
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'greenbin_to_pkg.yaml', 5)

    #Activating the vacuum gripper
    ur5.vaccum(True)

    # attach box to gripper
    ur5.attach_box('attach')

    print("uppp")
    # to lift the box up
    ur5.hard_set_joint_angles([3.001965295617115, -0.7678599563051787, 0.8727562554473742,-1.6756033726890767, -1.5707122165038765, -0.1395453900955168],5)

                
    url="https://spreadsheets.google.com/feeds/list/1xSQAM7_yopXE25Fsblj4me2-frBx9rzoMh8tJAiOgmU/2/public/full?alt=json"
    ssContent=requests.get(url).json()

    data_values = ssContent['feed']['entry'][i]['content'].values()
    val = data_values[0].encode("utf-8")
    str_values = list(val.split(","))
    data_separated=[]
    for str_value in str_values:
        each_val = str_value.split(":",1)
        each_val[1] = each_val[1][1:]
        data_separated.append(each_val)
    order_date = datetime.datetime.strptime(data_separated[3][1], '%Y-%m-%d %H:%M:%S')


    url1="https://spreadsheets.google.com/feeds/list/1xSQAM7_yopXE25Fsblj4me2-frBx9rzoMh8tJAiOgmU/3/public/full?alt=json"
    ssContent1=requests.get(url1).json()

    data_values1 = ssContent1['feed']['entry'][i]['content'].values()
    val1 = data_values1[0].encode("utf-8")
    str_values1 = list(val1.split(","))
    data_separated1=[]
    for str_value in str_values1:
        each_val = str_value.split(":",1)
        each_val[1] = each_val[1][1:]
        data_separated1.append(each_val)
    print("go to bin")
    # Moving the ur5 arm along with the attached box to the red bin
    if data_separated[5][1] == "HP":
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg_to_redbin.yaml', 5)
        current = "redbin"
        ur5.conveyor(100)
        ur5.attach_box('detach')
        ur5.vaccum(False)
        parameters = {"id":"OrdersShipped","Team Id":"VB#0566","Unique Id":"AiRiAkAk","Order ID":data_separated[2][1],"City":data_separated[7][1],"Item":"Medicine","Priority":"HP","Shipped Quantity":1,"Cost":450,"Shipped Status":"YES","Shipped Date and Time":datetime.datetime.now(),"Estimated Time of Delivery":"2021-01-23"}
        URL1 = "https://script.google.com/macros/s/AKfycbxEujrxYzn6uhcI9IKyHt-PumJxPJFYRxYtC3TRhW1fHgQ5yiFIjHIulg/exec"
        response = requests.get(URL1, params=parameters)

        now = datetime.datetime.now()
        diff = now - order_date
        l = list(divmod(diff.total_seconds(),60))
        diff_sec = l[0]*60 + l[1]
        parameters = {"id":"Dashboard","Team Id":"VB#0566","Unique Id":"AiRiAkAk","Order Id":data_separated[2][1],"Item":"Medicine","Priority":"HP","Quantity":1,"City":data_separated[7][1],"Longitude":data_separated[8][1],"Latitude":data_separated[9][1],"Order Dispatched":"YES","Order Shipped":"YES","Order Time":data_separated[3][1],"Dispatch Time":data_separated1[9][1],"Shipping Time": datetime.datetime.now(),"Time Taken":diff_sec}
        URL2 = "https://script.google.com/macros/s/AKfycbzXwIg573leL8Oy2aFIieRgqROp8Wfn1mT-_Ue864BuaZmVCvLgpb3w6A/exec"
        response = requests.get(URL2,params = parameters)
        # print(response.content)
    # Moving the ur5 arm along with the attached box to the yellow bin
    elif data_separated[5][1] == "MP":
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg_to_yellowbin.yaml', 5)
        current = "yellowbin"
        ur5.conveyor(100)
        ur5.attach_box('detach')
        ur5.vaccum(False)
        parameters = {"id":"OrdersShipped","Team Id":"VB#0566","Unique Id":"AiRiAkAk","Order ID":data_separated[2][1],"City":data_separated[7][1],"Item":"Food","Priority":"MP","Shipped Quantity":1,"Cost":250,"Shipped Status":"YES","Shipped Date and Time":datetime.datetime.now(),"Estimated Time of Delivery":"2021-01-25"}
        URL1 = "https://script.google.com/macros/s/AKfycbxEujrxYzn6uhcI9IKyHt-PumJxPJFYRxYtC3TRhW1fHgQ5yiFIjHIulg/exec"
        response = requests.get(URL1, params=parameters)

        now = datetime.datetime.now()
        diff = now - order_date
        l = list(divmod(diff.total_seconds(),60))
        diff_sec = l[0]*60 + l[1]
        parameters = {"id":"Dashboard","Team Id":"VB#0566","Unique Id":"AiRiAkAk","Order Id":data_separated[2][1],"Item":"Food","Priority":"MP","Quantity":1,"City":data_separated[7][1],"Longitude":data_separated[8][1],"Latitude":data_separated[9][1],"Order Dispatched":"YES","Order Shipped":"YES","Order Time":data_separated[3][1],"Dispatch Time":data_separated1[9][1],"Shipping Time": datetime.datetime.now(),"Time Taken":diff_sec}
        URL2 = "https://script.google.com/macros/s/AKfycbzXwIg573leL8Oy2aFIieRgqROp8Wfn1mT-_Ue864BuaZmVCvLgpb3w6A/exec"
        response = requests.get(URL2,params = parameters)
        # print(response.content)
    # Moving the ur5 arm along with the attached box to the green bin        
    elif data_separated[5][1] == "LP":
        ur5.moveit_hard_play_planned_path_from_file(ur5._file_path, 'pkg_to_greenbin.yaml', 5)
        current = "greenbin"
        ur5.conveyor(100)
        ur5.attach_box('detach')
        ur5.vaccum(False)
        parameters = {"id":"OrdersShipped","Team Id":"VB#0566","Unique Id":"AiRiAkAk","Order ID":data_separated[2][1],"City":data_separated[7][1],"Item":"Clothes","Priority":"LP","Shipped Quantity":1,"Cost":150,"Shipped Status":"YES","Shipped Date and Time":datetime.datetime.now(),"Estimated Time of Delivery":"2021-01-27"}
        URL1 = "https://script.google.com/macros/s/AKfycbxEujrxYzn6uhcI9IKyHt-PumJxPJFYRxYtC3TRhW1fHgQ5yiFIjHIulg/exec"
        response = requests.get(URL1, params=parameters)

        now = datetime.datetime.now()
        diff = now - order_date
        l = list(divmod(diff.total_seconds(),60))
        diff_sec = l[0]*60 + l[1]
        parameters = {"id":"Dashboard","Team Id":"VB#0566","Unique Id":"AiRiAkAk","Order Id":data_separated[2][1],"Item":"Clothes","Priority":"LP","Quantity":1,"City":data_separated[7][1],"Longitude":data_separated[8][1],"Latitude":data_separated[9][1],"Order Dispatched":"YES","Order Shipped":"YES","Order Time":data_separated[3][1],"Dispatch Time":data_separated1[9][1],"Shipping Time": datetime.datetime.now(),"Time Taken":diff_sec}
        URL2 = "https://script.google.com/macros/s/AKfycbzXwIg573leL8Oy2aFIieRgqROp8Wfn1mT-_Ue864BuaZmVCvLgpb3w6A/exec"
        response = requests.get(URL2,params = parameters)
        # print(response.content)

                
    # Drop the box into the red bin
    # ur5.attach_box('detach')
	        
    # deactivates the gripper
    # ur5.vaccum(False)
	        
    # removes box from the rviz
    # ur5.remove_box()
    i+=1

    

if __name__ == '__main__':
    main(sys.argv)
