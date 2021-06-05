#!/usr/bin/env python

# ROS Node - Simple Action Client - Turtle

import rospy
import actionlib
import time

from pkg_task1.msg import msgTurtleAction      # Message Class that is used by ROS Actions internally
from pkg_task1.msg import msgTurtleGoal         # Message Class that is used for Goal messages

from pkg_ros_iot_bridge.msg import msgRosIotAction  
from pkg_ros_iot_bridge.msg import msgRosIotGoal
from pkg_ros_iot_bridge.msg import msgRosIotResult

from pkg_iot_ros_bridge.msg import msgMqttSub    


i=0
f=0
class SimpleActionClientTurtle:
   
    # Constructor
    def __init__(self):
        self._ac = actionlib.SimpleActionClient('/action_turtle',
                                                msgTurtleAction)
        self.Iot_client=Send_IOT()
        self._ac.wait_for_server()
        rospy.loginfo("Action server is up, we can send new goals!")

    # Function to send Goals to Action Servers
    def send_goal(self, arg_dis, arg_angle):
        global i
        # Create Goal message for Simple Action Server
        goal = msgTurtleGoal(distance=arg_dis, angle=arg_angle)
        
        '''
            * done_cb is set to the function pointer of the function which should be called once 
                the Goal is processed by the Simple Action Server.

            * feedback_cb is set to the function pointer of the function which should be called while
                the goal is being processed by the Simple Action Server.
        ''' 
       
        self._ac.send_goal(goal, done_cb=self.done_callback,
                           feedback_cb=self.feedback_callback)
        i=i+1

        rospy.loginfo("Goal " + str(i)+ " has been sent.")

    # Function print result on Goal completion
    def done_callback(self, status, result):
        global f
        res=[]
        rospy.loginfo("Result is : " + str(result))
        res.append(result.final_x)
        res.append(result.final_y)
        res.append(result.final_theta)
        rospy.loginfo(res)
        self.Iot_client.send_goal_Iot("mqtt","pub","eyrc/AiRiAkAk/ros_to_iot",str(res))

    # Function to print feedback while Goal is being processed
    def feedback_callback(self, feedback):
        rospy.loginfo(feedback)

# Class used to publish the data(x,y,theta) to mqtt topic
class Send_IOT:

    #Constructor 
    def __init__(self):
        
        self._ac2=actionlib.ActionClient('/action_ros_iot',msgRosIotAction)
        #Dictionary to Store all the goal handels
        self._goal_handles={}  
        param_config_iot=rospy.get_param('config_iot')
        self._config_mqtt_pub_topic=param_config_iot['mqtt']['topic_pub']
        self._ac2.wait_for_server()
        rospy.loginfo("IOT Action server up.")   

    
    # Function responsible to send the data to mqtt
    def send_goal_Iot(self,arg_protocol,arg_mode,arg_topic,arg_message):
        goal_Iot=msgRosIotGoal()
        goal_Iot.protocol=arg_protocol
        goal_Iot.mode=arg_mode
        goal_Iot.topic=arg_topic
        goal_Iot.message=arg_message
        goal_handle=self._ac2.send_goal(goal_Iot,None,None)

        return goal_handle


# To check the data recieved from the mqtt topic
def call_back_begin(data):

    global f
    if data.topic == 'eyrc/AiRiAkAk/iot_to_ros' and data.message=='start':
        # As soon as "start" message is recieved from the mqtt, Flag register is changed to 1
        f=1

# Main Function

def main():
    # 1. Initialize ROS Node
    rospy.init_node('node_simple_action_client_turtle')

    # its subscribes the messages from the mqtt topic
    rospy.Subscriber('/ros_iot_bridge/mqtt/sub',msgMqttSub , call_back_begin)

    # infinite loop untill Flag register is 0
    while f == 0:
       pass

    # When flag is 1, turtle starts
    #rospy.logwarn(f)

    # 2. Create a object for Simple Action Client.
    obj_client = SimpleActionClientTurtle()

    # 3. Send Goals to Draw a Hexagon
    obj_client.send_goal(2, 0)
    rospy.sleep(7)
   
    obj_client.send_goal(2, 60)
    rospy.sleep(11)

    obj_client.send_goal(2, 60)
    rospy.sleep(11)

    obj_client.send_goal(2, 60)
    rospy.sleep(11)

    obj_client.send_goal(2, 60)
    rospy.sleep(11)

    obj_client.send_goal(2, 60)
    rospy.sleep(11)

    # 4. Loop forever
    rospy.spin()


if __name__ == '__main__':
    main()
