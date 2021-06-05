eYRC 2020-21: Vargi Bots (VB) {.menu-title}
=============================

[Objective](#objective)
=======================

The objective of this task is to implement a Warehouse Management System
to sort packages based on incoming customer orders from different
cities. To achieve this team needs to do the following things:

1.  Identify the colour of any 9 packages, three of each colour, on
    shelf using `Camera#1`. Either colour detection or QR
    decoding or combination of both can be used here.

    > **NOTE**: Hard-coding the colour of the packages is not allowed.

2.  As the packages are identified the teams must update the
    `inventory` sheet of
    `Inventory Management Spreadsheet` of the warehouse which is
    a Google Spreadsheet using the `ROS- IoT bridge` developed by
    the Teams in Task 1.

    > **NOTE**: Teams are allowed to pick and sort only those packages
    > that are updated in this `Inventory Sheet`.

3.  After one minute (Sim Time), a total of 9 orders will be published
    on `/eyrc/vb/<unique_id>/orders` MQTT Topic at different
    intervals. Using `ROS-IoT Bridge` teams need to get the
    orders from this MQTT Topic.

    > `<unique-id>: `This is a private ID which will be the eight
    > character string created by the teams during Task-1.

    > **NOTE**: You would have to update the `mqtt_unique_id`
    > field in `pkg_vb_sim/config/config_online_order.yaml` in
    > order to make the `node_place_online_order` to publish
    > orders on MQTT Topic: `/eyrc/vb/<unique_id>/orders`

4.  In case the `UR5#1` has multiple orders of different
    priorities to process, teams need to make sure that High Priority
    orders are processed as quickly as possible, then Medium Priority
    and then Low Priority orders are processed in order to score maximum
    points (refer grading section to learn more). Packages of
    `red` colour will have `High Priority` symbolizing
    Medicines, packages of `yellow` colour will have a
    `Medium Priority` symbolizing Food and packages of
    `green` colour have a `Low Priority` symbolizing
    Clothes.

5.  Once the package is placed on the `conveyor belt`, teams need
    to update the `Orders Dispatched` sheet in the
    `Inventory Management Spreadsheet` which should give the
    status of the packages picked up by the `UR5#1` Arm and send
    an email notification to the user.

6.  Once the `conveyor belt` takes the packages to
    `UR5#2`, the `UR5#2` Arm then needs to sort the
    packages based on the colour of the package identified by
    `Camera#1`. For eg. Red Package should go in the Red-Bin and
    so on.

7.  As the `UR5#2` Arm sorts the individual packages into the
    bins based on package colour, the teams will need to update the
    `Orders Shipped` sheet of the
    `Warehouse Inventory Mastersheet` which should give the
    status of the packages being picked and dropped in the bins by the
    `UR5#2` Arm.

8.  As the run is progressing Teams will have to update the
    `Warehouse Inventory Dashboard` in real-time. In the
    `Warehouse Inventory Mastersheet` teams can create a separate
    sheet called `Dashboard` which can show values from other
    sheet in the `Warehouse Inventory Mastersheet` spreadsheet.
    Teams can use this `Dashboard` sheet as a JSON endpoint to
    update the `Warehouse Inventory Dashboard` in real-time.

> **NOTE**: Refer the rulebook for more details.

[**](https://portal.e-yantra.org/storage/xyBeoIQDWX_vb/eyrc/task5/eyrc-task5-problem-statement.html "Previous chapter")
[**](https://portal.e-yantra.org/storage/xyBeoIQDWX_vb/eyrc/task5/eyrc-task5-ps-2.html "Next chapter")

[**](https://portal.e-yantra.org/storage/xyBeoIQDWX_vb/eyrc/task5/eyrc-task5-problem-statement.html "Previous chapter")
[**](https://portal.e-yantra.org/storage/xyBeoIQDWX_vb/eyrc/task5/eyrc-task5-ps-2.html "Next chapter")
