
eYRC 2020-21: Vargi Bots (VB) 
=============================

[Problem Statement](#problem-statement)
=======================================

-   **Objective**: The objective of this task is to sort any nine
    packages out of twelve packages in any order as quickly as possible.
    To achieve this team needs to do the following things.

    1.  Identify the colour of the packages on shelf using Camera\#1.
        Either colour detection or QR decoding or combination of both
        can be used here. **NOTE**: Hard-coding the colour of the
        packages is not allowed.

    2.  UR5\#1 would have to pick the packages from the shelf and place
        it on the conveyor belt.

    3.  Conveyor belt needs to take the packages to UR5\#2.

    4.  UR5\#2 then needs to sort the packages based on the colour of
        the package identified by Camera\#1. For eg. Red Package should
        go in the Red-Bin and so on.

        \

-   **Collision Avoidance**

    -   While sorting the packages the team needs to make sure that the
        UR5 Arms (UR5\#1 and UR5\#2) are not colliding with the conveyor
        belt, packages on conveyor belt, with the shelf, packages in the
        shelf, with the bins or with itself.

    -   Once a package is picked the team needs to make sure that the
        package is also not colliding with anything.

    -   You can use the concepts of Task-2 and Task-3 to implement
        collision avoidance.

\

-   **Cameras**

    -   In this task there are two logical cameras and one 2D camera.

    -   The role of the 2D Camera is to use Computer Vision techniques
        to identify the packages in the shelf compartments.

    -   The configuration of the packages is given as follows:

        1.  `packagen00` - first package first row.
        2.  `packagen01`- second package first row.
        3.  `packagen02` - third package first row.
        4.  `packagen10` - first package second row.
        5.  `packagen11` - second package second row.
        6.  `packagen12` - third package second row.
        7.  `packagen20` - first package third row.
        8.  `packagen21` - second package third row.
        9.  `packagen22` - third package third row.
        10. `packagen30` - first package fourth row.
        11. `packagen31` - second package fourth row.
        12. `packagen32` - third package fourth row.

\

-   **TF**
-   In this task do not rely on TF values as they can fluctuate since we
    are using two UR5 arms. We would suggest the teams to use Logical
    Camera\#2 feed to manually calculate the translations. You can do
    this by using the concepts of TF which you learned in Task-3.

\

-   **Simulation Time**

    -   In this task simulation time will be considered for grading. So,
        the teams must make sure to keep the simulation time as low as
        possible by quickly sorting the packages.

    -   **Strategies to decrease Simulation Time**

        -   The team can operate the conveyor belt at maximum speed so
            that packages reach the UR5\#2 faster.
        -   The team can make the UR5\#2 pick the packages from a moving
            conveyor belt.
        -   Or any other such strategies.

\

-   **ROS Packages required**:

    1.  `pkg_moveit_ur5_1` : The team will have to generate this
        package using MoveIt! Setup Assistant which will configure
        MoveIt! for the UR5\#1. For this use `ur5_1.urdf` in
        `pkg_vb_sim/urdf`.

        **NOTE:** This package is also available in Vargi Bots
        Simulation Packages as `pkg_moveit_ur5_1_config` with
        default configuration. You may refer that package to generate
        this package if you want to configure this your own way or you
        are free to use this package also.

    2.  `pkg_moveit_ur5_2` : The team will have to generate this
        package using MoveIt! Setup Assistant which will configure
        MoveIt! for the UR5\#2. For this use `ur5_2.urdf` in
        `pkg_vb_sim/urdf`.

        **NOTE:** This package is also available in Vargi Bots
        Simulation Packages as `pkg_moveit_ur5_2_config` with
        default configuration. You may refer that package to generate
        this package if you want to configure this your own way or you
        are free to use this package also.

    3.  `pkg_task4`: This is the ROS package in which the team is
        suppose to implement this task.

    4.  `Vargi Bots Simulation Packages`: These packages will
        have the simulation environment needed for this task.

[pkg\_task4](#pkg_task4)
------------------------

-   Similar to Task 3, in this Task teams are allowed to have their own
    structure for this package. This means that teams can have any
    number of nodes (Python Scripts **only**), message files, service
    files, action files, config files etc. in order to solve this task.

#### [`task4_solution.launch`](#task4_solutionlaunch)

-   It is compulsory for the teams to have a launch file called
    `task4_solution.launch` that will launch the simulation
    environment and all the necessary nodes that are required to solve
    this task. While evaluating we are going to use this launch file to
    run your implementation so make sure that this launch file is
    proper.

-   In this launch file you need to include the following in order to
    spawn all the necessary models and nodes for Task-4 simulation.

        <!-- Launch Task-4 Simulation Environment in Gazebo -->
        <include file="$(find pkg_vb_sim)/launch/task4_simulation.launch" />

-   You will also launch `two_ur5_move_group.launch` if you
    decide to use `pkg_moveit_ur5_1_config` and
    `pkg_moveit_ur5_2_config` instead of your own MoveIt!
    Configuration Packages for both the UR5 arms.

        <include file="$(find pkg_vb_sim)/launch/two_ur5_move_group.launch" />

-   If you are using your own `pkg_moveit_ur5_1` and
    `pkg_moveit_ur5_2` packages then you would have to include
    your own version of `two_ur5_move_group.launch` also in
    `pkg_task4`.

-   Properly comment this launch file. You need to add detailed
    description of all the nodes that you are going to include in this
    file as comments. For example,

        <!-- This node is for .....  -->
        <!-- It is also .....  -->
         <node name= "node_t4" pkg= "pkg_task4" type="node_t4.py" output="screen"/>

#### [`bag_files` folder](#bag_files-folder)

-   The team would have to create this folder in `pkg_task4` for
    storing the bag file.

-   Instructions to record bag file is given in Submission Instruction
    section.

\
 \

* * * * *

[**](https://portal.e-yantra.org/storage/xyBeoIQDWX_vb/eyrc/task4/eyrc-task4-learn.html "Previous chapter")
[**](https://portal.e-yantra.org/storage/xyBeoIQDWX_vb/eyrc/task4/eyrc-task4-expected-output.html "Next chapter")

[**](https://portal.e-yantra.org/storage/xyBeoIQDWX_vb/eyrc/task4/eyrc-task4-learn.html "Previous chapter")
[**](https://portal.e-yantra.org/storage/xyBeoIQDWX_vb/eyrc/task4/eyrc-task4-expected-output.html "Next chapter")
