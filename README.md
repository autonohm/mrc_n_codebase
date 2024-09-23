# Mobile Robotics Competition - Nuremberg

Welcome to the codebase repository for the MCR-N course!


# Development State!


# Overview

1. Basic Idea
2. Useful links
3. Hardware Description
4. Target Software Architecture
5. Example Robot Behavior
6. Uploading your Code
7. Using the Master Control Node



# 1. Basic Idea

Students shall program robots to navigate a pre-defined parcours.
With given hardware specifications (Praktikumsplattform), students should implement
robot localization, task following and navigation.

We provide them with a well-defined environment that they should use.
The goal is for the students to integrate their work into the given system.



# 2. Useful Links

TODo Remove: hier dann bitte die Slides und größeren Files rein:

https://1drv.ms/f/s!Av9EgKpWKlj-gp1GSEwbTssOGBg36A?e=d9G274


## Windows Sublinux

## ROS 2 Humble

## ROS 2 Navigation Stack

## Python Simulator ROS 2

[Python Simulator by Prof May](https://github.com/autonohm/ohm_mecanum_sim/tree/ros2)



# 3. Hardware Description



# 4. Target Software Architecture

## General Overview

![](https://github.com/autonohm/mcr_n_codebase/blob/main/images/basic_architecture_v1.png)

## Simulation

![](https://github.com/autonohm/mcr_n_codebase/blob/main/images/basic_sim_v1.png)

## Navigation
![](https://github.com/autonohm/mcr_n_codebase/blob/main/images/basic_navigation_v1.png)


# 5. Example Robot Behavior

PROVIDE LINK

# 6. How to upload your code

# 7. The Master Control Node


## 7.1 Starting the Server

The master control node acts as the task server. Once you built this pkg, 
you can start the node using the provided launchfile:

```
your_user@your_machine:~$ roslaunch mrc_n_codebase master_control.launch
```

The output of the node should look like this:

```
..

SUMMARY
========

PARAMETERS
 * /master_control_server/folder_path: /home/your_machine/mrc_w...
 * /master_control_server/get_plan_topic: move_base/make_plan
 * /master_control_server/save_current_req_topic: save_current_req
 * /master_control_server/start_test_topic: start_test
 * /rosdistro: noetic
 * /rosversion: 1.16.0

NODES
  /
    master_control_server (mrc_n_codebase/master_control_node.py)

auto-starting new master
process[master]: started with pid [31269]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 73507568-6d19-11ef-84f0-145afc8ede9d
process[rosout-1]: started with pid [31296]
started core service [/rosout]
process[master_control_server-2]: started with pid [31300]
[INFO] [1725714156.810645]: [MCS]       Reading files from folder: /home/your_machine/mrc_ws/src/mrc_n_codebase/master_control_tasks
[INFO] [1725714156.812165]: [MCS]       Reading File from /home/marco/your_machine/src/mrc_n_codebase/master_control_tasks/test1.txt
[INFO] [1725714156.813956]: [MCS]       Successfully read tasks from 1 files and found the target [test1]
[INFO] [1725714156.819780]: [MCS]       Started..

```

## 7.2 Connecting to the Server

Robots must connect to the server using the provided 'mcs_connect' service,
providing their name.

From the terminal, this can be done using:

```
your_user@your_machine:~$ rosservice call /master_control/connect "robot_name: 'test_robot'" 
success: True
```

The node should output the following:

```
[INFO] [1725714158.608101]: [MCS]       Robot [test_robot] has connected!success: True
```

However, you will need to connect to the server from your own statemachine file!

## 7.3 Getting Tasks from the Server

The target task from the server can be queried using the provided service:

```
your_user@your_machine:~$ rosservice call /master_control/get_tasks "robot_name: 'test_robot'"
success: True
goals:
..
```

The node should output the following:

```
[INFO] [1725714162.910627]: [MCS]       Returning Task [test1] including [2] goals
```

If you did not use the connect service or if you provide the wrong robot name, 
the node will reject the request:

```
your_user@your_machine:~$ rosservice call /master_control/get_tasks "robot_name: 'abc'" 
success: False
goals: []
```

The node should output the following:

```
[ERROR] [1725715131.461621]: [MCS]      This robot [abc] has not connected before. Rejecting task request!
```






