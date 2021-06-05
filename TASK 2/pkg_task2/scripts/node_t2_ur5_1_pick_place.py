#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math


from pkg_vb_sim.srv import vacuumGripper

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
        

        # Current State of the Robot is needed to add box to planning scene



        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')


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


    def add_box(self):
        #Function to add a box to the planning scene
        scene = self._scene
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.x = 0.036257
        box_pose.pose.position.y = 0.456784
        box_pose.pose.position.z = 1.965725
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
    
    # Destructor
    
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')





#This is the main function of the program
def main():

    #This is used to create an object of the Ur5Moveit class

    ur5 = Ur5Moveit()
    #Specifying the coordinates of the joints in the planning scene

    lst_joint_angles_1 = [-1.197857,-1.75888,-0.235201,-1.146466,1.1985488,-0.001088]
    lst_joint_angles_2 = [-0.58525,-1.82878,0.229299,0.212344,1.8405350,-0.96025266]
    lst_joint_angles_3 = [0.29404,-2.686825,-0.82450,0.36806,-0.2949,0.000982]

    
    #Functions to execute the required tasks

    while not rospy.is_shutdown():
        #Function to add a box to the planning scene
        ur5.add_box()
        #Function to set the joints to the desired co-ordinates
        ur5.set_joint_angles(lst_joint_angles_1)
        rospy.sleep(2)
        #Function to activate the vacuum gripper
        ur5.vaccum(True)
        #Function to attach the box to the vacuum gripper
        ur5.attach_box('attach')
        #Function to set the joint angles to the required co-ordinates
        ur5.set_joint_angles(lst_joint_angles_2)
        ur5.set_joint_angles(lst_joint_angles_3)
        #Deactivate the vacuum gripper
        ur5.vaccum(False)
        #Detach the box from the vacuum gripper
        ur5.attach_box('detach')
        rospy.sleep(2)
        #Return the arm to it's initial coordinates
        ur5.go_to_predefined_pose("allZeros")
        rospy.sleep(2)
        rospy.spin() 
    #Delete the ur5 object created
    del ur5


if __name__ == '__main__':
    main()
