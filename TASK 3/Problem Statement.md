eYRC 2020-21: Vargi Bots (VB) {.menu-title}
=============================

[Problem Statement](#problem-statement)
=======================================

-   **Objective**: The objective of this task is to sort the packages as
    quickly as possible using the same UR5 (UR5\#1) that was used in
    Task-2.

    ![](Problem%20Statement%20-%20eYRC%202020-21%20Vargi%20Bots%20(VB)_files/T3_sim_launch.png)

-   **Sorting**

    -   The UR5 (UR5\#1) needs to pick the packages from the conveyor
        belt and place it in the correct bin. Red-Package will go in Red
        Bin, Blue-Package in Blue Bin and Green-Package in Green Bin.
-   **Collision Avoidance**

    -   While sorting the packages the team needs to make sure that the
        UR5 (UR5\#1) is not colliding with the conveyor belt, packages
        on conveyor belt, the bins or with itself.

    -   Once a package is picked the team needs to make sure that the
        package is also not colliding with anything.

    -   You can use the concepts of Task-2 to implement collision
        avoidance.

-   **Logical Camera and Conveyor Belt**

    -   The team needs to use the feed from Logical Camera in this task
        to detect packages.

    -   The team would also have to control the conveyor belt in order
        to make the packages reach the UR5 (UR5\#1).

-   **TF**

    -   Teams may have to use TF of the package and End-Effector to make
        the EE go to the package.
-   **Simulation Time**

    -   In this task simulation time will be considered for grading. So,
        the teams must make sure to keep the simulation time as low as
        possible by quickly sorting the package.

    -   **Strategies to decrease Simulation Time**

        -   The team can operate the conveyor belt at maximum speed so
            that packages reach the UR5 faster.
        -   The team can make the UR5 pick the packages from a moving
            conveyor belt.
        -   Or any other such strategies.

\

-   **ROS Packages required**:

    1.  `pkg_moveit_ur5_1` : The team will have to generate this
        package using MoveIt! Setup Assistant which will configure
        MoveIt! for the UR5 (UR5\#1). You may use the same
        `pkg_moveit_ur5_1` which you have generated for Task-2.

    2.  `pkg_task3`: This is the ROS package in which the team is
        suppose to implement this task.

    3.  Vargi Bots Simulation Packages: These packages will have the
        simulation environment needed for this task.

[pkg\_task3](#pkg_task3)
------------------------

-   Unlike previous tasks in this tasks teams are allowed to have their
    own structure for this package. This means that teams can have any
    number of nodes (Python Scripts **only**), message files, service
    files, action files, config files etc. in order to solve this task.

#### [`task3_solution.launch`](#task3_solutionlaunch)

-   It is compulsory for the teams to have a launch file called
    `task3_solution.launch` that will launch the simulation
    environment and all the necessary nodes that are required to solve
    this task. While evaluating we are going to use this launch file to
    run your implementation so make sure that this launch file is
    proper.

-   In this launch file you need to include the following in order to
    spawn all the necessary models and nodes for Task-3 simulation.

        <!-- Launch Task-3 Simulation Environment in Gazebo -->
        <include file="$(find pkg_vb_sim)/launch/task3_simulation.launch" />

-   Properly comment this launch file. You need to add detailed
    description of all the nodes that you are going to include in this
    file as comments. For example,

        <!-- This node is for .....  -->
        <!-- It is also .....  -->
         <node name= "node_t3" pkg= "pkg_task3" type="node_t3.py" output="screen"/>

#### [`bag_files` folder](#bag_files-folder)

-   The team would have to create this folder in `pkg_task3` for
    storing the bag file.

-   Instructions to record bag file is given in Submission Instruction
    section.

\
 \

* * * * *

[**](https://portal.e-yantra.org/storage/xyBeoIQDWX_vb/eyrc/task3/eyrc-task3-learn.html "Previous chapter")
[**](https://portal.e-yantra.org/storage/xyBeoIQDWX_vb/eyrc/task3/eyrc-task3-expected-output.html "Next chapter")

[**](https://portal.e-yantra.org/storage/xyBeoIQDWX_vb/eyrc/task3/eyrc-task3-learn.html "Previous chapter")
[**](https://portal.e-yantra.org/storage/xyBeoIQDWX_vb/eyrc/task3/eyrc-task3-expected-output.html "Next chapter")
