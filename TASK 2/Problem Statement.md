eYRC 2020-21: Vargi Bots (VB) {.menu-title}
=============================

[Problem Statement](#problem-statement)
=======================================

-   The objective of this task is to implement pick and place using one
    UR5 (UR5\#1) in Gazebo.

    ![](Problem%20Statement%20-%20eYRC%202020-21%20Vargi%20Bots%20(VB)_files/vb-sim.png)

-   The UR5 (UR5\#1) needs to pick the blue package from the shelf and
    place it in the bin next to it.

-   While doing this the team needs to make sure that the UR5 (UR5\#1)
    is not colliding with the shelf, the bin or with itself.

-   After the blue package is picked the team needs to make sure that
    the package also does not collide with the shelf.

-   To do this collision avoidance the team can add the shelf and the
    box in the MoveIt! Planning Scene for the Planner.

-   Teams will need the following packages for this task:

    1.  `pkg_moveit_ur5_1` : The team will have to generate this
        package using MoveIt! Setup Assistant which will configure
        MoveIt! for the UR5 (UR5\#1).

    2.  `pkg_task2`: This is the package in which you will
        implement the pick and place and **the package the team needs to
        submit**.

    3.  Vargi Bots Simulation Packages: These packages will have the
        simulation environment needed for this task.

[pkg\_task2](#pkg_task2)
------------------------

![](Problem%20Statement%20-%20eYRC%202020-21%20Vargi%20Bots%20(VB)_files/t2-sub-tree.png)

-   This is the package that the team needs to submit.

-   `pkg_task2`: This package will consist of `scripts`,
    `launch` , `config` and a `bag_files` folder.

-   In your `scripts` folder, you need to write a ROS Node
    `node_t2_ur5_1_pick_place.py` which will do the following in
    order:

    1.  Make the end effector go to the package on the shelf while
        avoiding collision with the shelf.

    2.  Activate the vacuum gripper to make UR5 latch on the the
        package.

    3.  Make the end-effector go near the bin to drop the package in the
        bin while avoiding collision of the **package and UR5** with the
        shelf.

    4.  Deactivate the vacuum gripper to detach the package from the
        vacuum gripper.

    5.  Make all the joint angles zero while avoiding any collision.

-   Within the `config` folder, there should be a folder called
    `rviz` containing a scene file with the shelf in the Planning
    Scene of MoveIt!. Name this file `ur5_1_shelf.scene`.

-   `launch` folder should contain `task2.lauch` which
    should launch all the necessary nodes, configs and simulation
    environment needed for Task-2.

    Your launch file should be something like the following and need not
    be the same.

        <launch>
            
            <!-- Spawn Task-2 Models in Gazebo -->
            <include file="$(find pkg_vb_sim)/launch/task2_world.launch" />

            <include file = "$(find pkg_vb_sim)/launch/gazebo_spawn_ur5_1.launch" >
                <arg name="arg_x" value="0.04" />
                <arg name="arg_y" value="0.76" />
                <arg name="arg_z" value="0.05" />
                <arg name="arg_R" value="0.0" />
                <arg name="arg_P" value="0.0" />
                <arg name="arg_Y" value="3.14" />
            </include>

            
            <!-- Launch Moveit move_group Node -->
            <include file = "$(find pkg_moveit_ur5_1)/launch/move_group.launch" /> 

            <!-- Run RViz with Moveit! Plugin -->
            <arg name="use_rviz" default="true" />

            <include file="$(find pkg_moveit_ur5_1)/launch/moveit_rviz.launch" if="$(arg use_rviz)">
                <arg name="rviz_config" value="$(find pkg_moveit_ur5_1)/launch/moveit.rviz"/>
            </include>
         
            <!-- Add Shelf to Moveit! Planning Scene in RViz -->
            <arg name="scene_file" default="$(find pkg_task2)/config/rviz/ur5_1_shelf.scene"/> 
            <node name = "moveit_publish_scene_from_text" pkg= "moveit_ros_planning" type = "moveit_publish_scene_from_text" args= "$(arg scene_file)"/>

            <!-- Run Task-2 Pick Place ROS Node -->
            <node name= "node_t2_ur5_1_pick_place" pkg= "pkg_task2" type="node_t2_ur5_1_pick_place.py" output="screen"/>
            
            <!-- Recording Bag File for Submission -->
            <arg name="record" default="false"/>
            <arg name="rec_name" default="task2.bag"/>

            <group if="$(arg record)">
                <node name="rosbag_record_pick" pkg="rosbag" type="record"
                 args="record -O $(find pkg_task2)/bag_files/$(arg rec_name) --chunksize=10 /eyrc/vb/ur5_1/vacuum_gripper/logical_camera/ur5_1" output="screen"/>
           </group>

        </launch>

    -   If you want to record the bag file needed for submission you
        would have to run the following command.

            roslaunch pkg_task2 task2.launch record:=true rec_name:=t2.b

    -   If you just want to run your implementation without recording a
        bag file. Do the following.

            roslaunch pkg_task2 task2.launch

\
 \

* * * * *

[**](https://portal.e-yantra.org/storage/xyBeoIQDWX_vb/eyrc/task2/eyrc-task2-learn.html "Previous chapter")
[**](https://portal.e-yantra.org/storage/xyBeoIQDWX_vb/eyrc/task2/eyrc-task2-expected-output.html "Next chapter")

[**](https://portal.e-yantra.org/storage/xyBeoIQDWX_vb/eyrc/task2/eyrc-task2-learn.html "Previous chapter")
[**](https://portal.e-yantra.org/storage/xyBeoIQDWX_vb/eyrc/task2/eyrc-task2-expected-output.html "Next chapter")
