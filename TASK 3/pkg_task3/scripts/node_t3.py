#! /usr/bin/env python

import rospy
import sys
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


# Creating Ur5Moveit class
class Ur5Moveit:
    i=1
    # Constructor
    def __init__(self):

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        rospy.loginfo(self._scene)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        rospy.loginfo(self._group_names)
        self._box_name = 'box'
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

   
    
    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # Function for translation by waypoints

        # 1. Create a empty list to hold waypoints
        waypoints = []

        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)
        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)  
        wpose.position.y = waypoints[0].position.y + (trans_y)  
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5


        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))


        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        # https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/?answer=257488#post-id-257488
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    def go_to_pose(self, arg_pose):
        #Function to move the arm to the desired pose in the planning scene

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)
        
        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() " +str(self.i)+ " Success" + '\033[0m')
            self.i=self.i+1        
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() " + str(self.i)+ " Failed. Solution for Pose not Found." + '\033[0m')
            self.i=self.i+1    
        return flag_plan


    def set_joint_angles(self, arg_list_joint_angles):
        #Function to set the joint angles

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)
        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')
        return flag_plan
    
    def vaccum(self,condition):
        #Function to activate/deactivate the vacuum gripper

        rospy.wait_for_service('/eyrc/vb/ur5_1/activate_vacuum_gripper')
        try:
            self.gripper = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
            result=self.gripper(condition)

        #return gripper.result
        except Exception as e:
            rospy.logerr("Service call failed: %s"%e)

    def conveyor(self,value):
        #Function to set_power to conveyor

        rospy.wait_for_service('/eyrc/vb/conveyor/set_power')
        try:
            self.conveyor = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)
            result=self.conveyor(value)

        #return gripper.result
        except Exception as e:
            rospy.logerr("Service call failed: %s"%e)


    def add_box(self,x,y,z):
        #Function to add a box to the planning scene

        scene = self._scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = x
        box_pose.pose.position.y = y
        box_pose.pose.position.z = z
        scene.add_box(self._box_name, box_pose, size=(0.15, 0.15, 0.15))
 
    def attach_box(self,condition):
        #Function to attach the box to the vacuum gripper

        self._scene = moveit_commander.PlanningSceneInterface()
        grasping_group = 'ur5_1_planning_group'
        touch_links = self._robot.get_link_names(group=grasping_group)
        if condition == 'attach':
            self._scene.attach_box(self._eef_link, self._box_name, touch_links=touch_links)
        elif condition == 'detach':
            self._scene.remove_attached_object(self._eef_link, name=self._box_name)

    def remove_box(self, timeout=4):
	#Function to remove box from the planning scene

        box_name = self._box_name
        scene = self._scene
        scene.remove_world_object(box_name)
    	


    def go_to_predefined_pose(self, arg_pose_name):
        #Function to move the arm to the desired pose

        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name) + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name) + '\033[0m')
    
    # Destructor - Deleting the constructor
    
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

    #Getting coordinates for the green box
    def func_tf2_print(self, arg_frame_1, arg_frame_2):
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
            rospy.logerr("TF2 error")
            return L

    #Getting coordinates for blue box
    def func_tf3_print(self, arg_frame_1, arg_frame_2):
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
            rospy.logerr("TF3 error")
            return L


#This is the main function of the program

def main():
	
    #This is used to create an object of the Ur5Moveit class

    ur5 = Ur5Moveit()
    
    box_length = 0.15               # Length of the Package
    vacuum_gripper_width = 0.115    # Vacuum Gripper Width
    delta = vacuum_gripper_width + (box_length/2)  # 0.19
    

    #Position of ur5 arm on conveyor

    ur5_home_pose = geometry_msgs.msg.Pose()
    ur5_home_pose.position.x = -0.8
    ur5_home_pose.position.y = 0
    ur5_home_pose.position.z = 1 + vacuum_gripper_width + (box_length/2)

    # This to keep EE parallel to Ground Plane
    ur5_home_pose.orientation.x = -0.5
    ur5_home_pose.orientation.y = -0.5
    ur5_home_pose.orientation.z = 0.5
    ur5_home_pose.orientation.w = 0.5
    ####################################

    # created object of tfEcho to get the coordinates
    my_tf = tfEcho()
    reference_frame = "world"
    target_frame1 = "logical_camera_2_packagen1_frame"
    target_frame2 = "logical_camera_2_packagen2_frame"
    target_frame3 = "logical_camera_2_packagen3_frame"

    rospy.sleep(10)
    ur5.conveyor(70)
    # to go to red box position on conveyor
    ur5.go_to_pose(ur5_home_pose)

    # sets the conveyor speed to 100
    ur5.conveyor(100)

    #counter
    i = 0

    while not rospy.is_shutdown():
        
        # stores coordinates of red box
        val=my_tf.func_tf1_print(reference_frame, target_frame1)

	# if box comes under the logical camera
	if val[0] == 'True' and i==0:

            # if box comes under the logical camera
	    if val[2] <= 0.15:

		# sets the conveyor speed to 0
		ur5.conveyor(0)

		val=my_tf.func_tf1_print(reference_frame, target_frame1)

		# planning path to red box
		ur5_red_home_pose = geometry_msgs.msg.Pose()
 	        ur5_red_home_pose.position.x = val[1]
 	        ur5_red_home_pose.position.y = 0
	        ur5_red_home_pose.position.z = 1 + vacuum_gripper_width + (box_length/2)
	        
 	        # This to keep EE parallel to Ground Plane
	        ur5_red_home_pose.orientation.x = -0.5
	        ur5_red_home_pose.orientation.y = -0.5
    	        ur5_red_home_pose.orientation.z = 0.5
	        ur5_red_home_pose.orientation.w = 0.5

                #Adding box to the rviz planning scene
		ur5.add_box(val[1],val[2],val[3])

                #Translating the ur5 arm to the position of red box
		ur5.go_to_pose(ur5_red_home_pose)

                #Activating the vacuum gripper
		ur5.vaccum(True)

		# attach box to gripper
		ur5.attach_box('attach')

		# to lift the box up
                ur5.ee_cartesian_translation(0, 0, 0.1)

	        #Resuming the conveyor belt
		ur5.conveyor(40)
                
                #Set joint angles
		angles = [1.23,-1.014,1.580,-2.136,-1.570,-1.904]
                
                #Moving the ur5 arm along with the attached box to the red bin
		ur5.set_joint_angles(angles)

		# sets the conveyor speed to 100
 		ur5.conveyor(100)
                
		# Drop the box into the red bin
	    	ur5.attach_box('detach')
	        
		# deactivates the gripper
		ur5.vaccum(False)
	        
		# removes box from the rviz
	        ur5.remove_box()
		
		#counter
		i+=1

        # stores coordinates of green box
        val1=my_tf.func_tf2_print(reference_frame, target_frame2)

        # if box comes under the logical camera
	if val1[0] == 'True' and i==1:
            
	    # if box comes under the logical camera
	    if val1[2] <= 0.15:
	        
	        #Set the conveyor speed to 0
		ur5.conveyor(0)
	        
		val1=my_tf.func_tf2_print(reference_frame, target_frame2)

		# planning path to green box	        
		ur5_green_home_pose = geometry_msgs.msg.Pose()
	        ur5_green_home_pose.position.x = val1[1]
	        ur5_green_home_pose.position.y = 0
	        ur5_green_home_pose.position.z = 1 + vacuum_gripper_width + (box_length/2)
	        
	        # This to keep EE parallel to Ground Plane
	        ur5_green_home_pose.orientation.x = -0.5
	        ur5_green_home_pose.orientation.y = -0.5
	        ur5_green_home_pose.orientation.z = 0.5
	        ur5_green_home_pose.orientation.w = 0.5

	        #Adding the green box to the rviz planning scene
		ur5.add_box(val1[1],val1[2],val1[3])
	 
 	        #Translating the ur5 arm to the position of the green box
	        ur5.go_to_pose(ur5_green_home_pose)
 
	        #Activating the vacuum gripper
		ur5.vaccum(True)

                #Attaching the box with the end-effector of ur5 arm
		ur5.attach_box('attach')

		# to lift the box up
	        ur5.ee_cartesian_translation(0, 0, 0.1)

	        #Resuming the conveyor belt
		ur5.conveyor(40)

	        #Specying the joint angles for translation
		angles = [-0.106,-0.820,1.240,-1.991,-1.57,3.034]

	        #Translating the ur5 arm with the attached box to the green bin
		ur5.set_joint_angles(angles)

		# sets the conveyor speed to 100
 		ur5.conveyor(100)

	        #Detaching the box over the green bin
	    	ur5.attach_box('detach')

	        #Deactivating the vacuum gripper
		ur5.vaccum(False)

	        #Removing the box from the Rviz planning scene
	        ur5.remove_box()
		
	        #counter
		i+=1

	#Storing the coordinates of the blue box
        val2=my_tf.func_tf3_print(reference_frame, target_frame3)

	#if the box comes under logical camera
	if val2[0] == 'True' and i==2:
	
	    #if the box comes under logical camera
	    if val2[2] <= 0.15:

	        #Stopping the conveyor
		ur5.conveyor(0)
	
		val2=my_tf.func_tf3_print(reference_frame, target_frame3)

		# planning path to blue box
	        ur5_blue_home_pose = geometry_msgs.msg.Pose()
	        ur5_blue_home_pose.position.x = val2[1]
	        ur5_blue_home_pose.position.y = 0
	        ur5_blue_home_pose.position.z = 1 + vacuum_gripper_width + (box_length/2)
	        
	        # This to keep EE parallel to Ground Plane
	        ur5_blue_home_pose.orientation.x = -0.5
    	        ur5_blue_home_pose.orientation.y = -0.5
	        ur5_blue_home_pose.orientation.z = 0.5
	        ur5_blue_home_pose.orientation.w = 0.5

	        #Adding the box to the Rviz planning scene
		ur5.add_box(val2[1],val2[2],val2[3])
		
	        #Translating the ur5 arm to the position of blue box
		ur5.go_to_pose(ur5_blue_home_pose)

	        #Activating the vacuum gripper
		ur5.vaccum(True)

                #Attaching the blue box to the end-effector of the ur5 arm
		ur5.attach_box('attach')

		# to lift the box up
	        ur5.ee_cartesian_translation(0, 0, 0.1)

 	        #Setting the joint angles 
		angles = [1.800,-2.110,-1.607,-0.993,1.570,1.800]

		#Translating the ur5 arm and the attached box to the blue bin
		ur5.set_joint_angles(angles)

                #Detaching the box over the blue bin
	    	ur5.attach_box('detach')

	        #Removig the box in the rviz planning scene
	        ur5.remove_box()

	        #Deactivating the vacuum gripper
		ur5.vaccum(False)

                # sets the conveyor speed to 100
 		ur5.conveyor(100)

	        #Counter
		i+=1

    rospy.spin()
	    
    #Delete the ur5 object created
    del ur5

    #Delete the my_tf object created
    del my_tf

if __name__ == '__main__':
    main()
