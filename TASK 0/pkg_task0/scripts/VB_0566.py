#!/usr/bin/env python

''' First we need to import the packages used on our script.The rospy library is the ros python library, it contains the basic functions, like creating a node, getting time and creating a publisher.The geometry_msgs contains the variable type Twist that will be used, and The turtlesim.msg contains the Pose message type, which is the one published to the topic '/turtle1/pose' '''

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
PI = 3.1415926535897    # default value of pi
x_initial = 0           # initial x-cordinate of turtle
y_initial = 0           # initial y-cordinate of turtle
flag = 0                # to ignnore the repeated coordinates of turtle during the start
x_current = 0           # current x-cordinate of turtle
y_current = 0           # current y-cordinate of turtle
time_current = 0        # current time
t_initial = 0           # initial time


# to get the position angle of the turtle
def pose_callback(pose_msg):
    # scope of these variables are defined throughout the whole program
    global x_initial, x_current, time_initial
    global y_initial, y_current, flag
    # defination of initial time and converted to seconds
    time_current = rospy.Time.now().to_sec()
    if flag == 0:
        # initial coordinates of the turtle is stored
        x_initial = round(pose_msg.x)
        y_initial = round(pose_msg.y)
        flag = flag + 1
    if x_initial != round(pose_msg.x) and y_initial != round(pose_msg.y):
        # to ignnore the repeated coordinates of turtle during the start, flag is incremented to 3
        flag = 3
    # to record the log info and print the duration of turtle travelled
    rospy.loginfo("Moving in circle")
    print(time_current-time_initial)
    # to store the current coordinates of the turtle
    x_current = round(pose_msg.x)
    y_current = round(pose_msg.y)


# defines the motion of the turtle
def revolve_turtle():
    # scope of these variables are defined throughout the whole program
    global x_initial, x_current, time_initial
    global y_initial, y_current, flag
    # Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)

    # used to send the data to the topic
    velocity_publisher = \
        rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    # used to recieve information from the same topic
    sub = rospy.Subscriber("/turtle1/pose", Pose, pose_callback, None, 10)
    time_initial = rospy.Time.now().to_sec()
    # hardcoding the speed and radius
    speed = 1.1
    radius = 1

    # Converting from angles to radians
    # angular_speed = speed/radius

    # combination of linear and angular motion
    vel_msg.linear.x = speed
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = speed/radius

    # Moving the turtle in circular path untill it reaches the initial position
    while True:
        velocity_publisher.publish(vel_msg)
        # to check if the turtle has reached to initial position after 1 round
        a = ((flag == 3) and (x_initial == x_current and y_initial == y_current))
        if a is True:
            rospy.loginfo("Goal reached")
            break
    # Forcing our robot to stop by making linear and angular speed to zero
    vel_msg.angular.z = 0
    vel_msg.linear.x = 0
    velocity_publisher.publish(vel_msg)
    exit()
    # the statement guarantees that if we press ctrl+c our code will stop
    rospy.spin()
if __name__ == '__main__':
    try:
        # calling the revolve_turtle function
        revolve_turtle()
    except rospy.ROSInterruptException:
        pass

