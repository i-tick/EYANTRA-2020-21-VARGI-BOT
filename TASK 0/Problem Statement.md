eYRC 2020-21: Vargi Bots (VB) {.menu-title}
=============================

[Problem Statement](#problem-statement)
=======================================

-   The objective of the task is to move the turtle inside the
    [**turtlesim**](http://wiki.ros.org/turtlesim) window in a circle
    and stop at its initial location.

-   Teams are supposed to do this by creating a nodes name,
    **`/node_turtle_revolve`** within a python script,
    `node_turtle_revolve.py`.

[Procedure](#procedure)
-----------------------

1.  First, create a package named `pkg_task0` , within your catkin
    workspace. Once done, compile and source the packages.

        cd ~/catkin_ws
        catkin build
        source devel/setup.bash

2.  Within this package, you should have a `scripts`  folder
    inside which you'll create a python script, named
    `node_turtle_revolve.py` .

    > Fill the script with [proper programming
    > ethics](https://alvinalexander.com/programming/clean-code-quotes-robert-c-martin/).
    > Doing this will help us understand your code better and quicker
    > than usual.

3.  After completing the python script. Make it executable, if it isn't
    already. To do that, enter the following code.

        chmod +x ~/catkin_ws/src/pkg_task0/scripts/node_turtle_revolve.py

4.  Before executing make sure that `roscore`  is running along
    with `turtlesim_node` . You can either run them in separate
    terminals or simply create a `task0.launch`  file inside the
    `~/catkin_ws/src/pkg_task0/launch/`  folder. Launch file can
    run multiple nodes unlike a python/cpp script. Run the launch file,
    enter,

        roslaunch pkg_task0 task0.launch 

    -   This should run three processes in parallel.
        -   `roscore` 
        -   `turtlesim_node` 
        -   `node_turtle_revolve.py` 

[Hints](#hints)
---------------

1.  The turtle needs to move in a circular motion with a certain radius.
    This radius should be sufficient to fit within the turtlesim window.
    But making it rotate in a circular manner, with only velocities to
    control is something to think about.

2.  Use linear velocity as well as angular velocity with some
    combination to get this done.

3.  Keep tracking the distance travelled so as to know when to stop. You
    can refer to [Overview of
    rospy](https://portal.e-yantra.org/storage/xyBeoIQDWX_vb/learn/ros-basics/learn-ros-nodes-rospy.html)
    for more hints.

> Next Read: [Expected
> Output](https://portal.e-yantra.org/storage/xyBeoIQDWX_vb/eyrc/task0/eyrc-task0-expected-output.html)

* * * * *

