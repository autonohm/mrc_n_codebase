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

Use [this template repository](https://github.com/autonohm/mrc_n_group_0) to fork and create your own repo for your group.
Make sure to set it to private and to add your colleagues.

Also add us so we can access and review your work:
- Marco Masannek (marcomasa)
- Dong Wang (wayne-dwa)
- Rolf Schmidt (rs-waid)
- Hannes Haag (todo)

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
 * /master_control_server/max_goal_deviation_angular: 0.2
 * /master_control_server/max_goal_deviation_linear: 0.1
 * /master_control_server/robot_frame_id: base_link
 * /master_control_server/target_task_name: mockup_test
 * /master_control_server/topic_confirm_goal_reached: master_control/co...
 * /master_control_server/topic_connect: master_control/co...
 * /master_control_server/topic_get_tasks: master_control/ge...
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
[INFO] [1727103559.011682]: [MCS]       Reading files from folder: /home/marco/mrc_ws/src/mrc_n_codebase/master_control_tasks
[INFO] [1727103559.013987]: [MCS]       Reading File from /home/marco/mrc_ws/src/mrc_n_codebase/master_control_tasks/mockup_test.txt
[INFO] [1727103559.015688]: [MCS]       Successfully read tasks from 1 files and found the target task [mockup_test]
[INFO] [1727103559.036435]: [MCS]       Started..

```

If you open RVIZ and add the topic type 'TF', you should be able to see the goal poses visualized:

![](https://github.com/autonohm/mcr_n_codebase/blob/main/images/mc_pose_example.png)


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
[INFO] [1725714158.608101]: [MCS]       Robot [test_robot] has connected!
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
[INFO] [1725714162.910627]: [MCS]       Returning Task [mockup_test] including [2] goals
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

## 7.4 Confirming a reached goal

Once your robot has reached a goal, you are supposed to confirm this with the server.
You just have to tell the server which goal pose you think you reached, and the server 
will check wether or not you are within the given allowed deviation.

```
your_user@your_machine:~$ rosservice call /master_control/confirm_goal_reached "goal_name: 'start'"
success: True
```

If that is the case, your exact score (consisting of the time to reach and the achieved precision) 
will be saved. You will get your final score once you set your status to finished.

```
[INFO] [1727103660.134773]: [MCS]       Trying to confirm reaching the goal pose [start]..
[INFO] [1727103660.136486]: [MCS]       Successfully reached the goal pose [start]!
[INFO] [1727103660.137412]: [MCS]       ------
[INFO] [1727103660.138331]: [MCS]       Goal Name:   start
[INFO] [1727103660.139200]: [MCS]       reached:     True
[INFO] [1727103660.140409]: [MCS]       time in s:   9.901
[INFO] [1727103660.141522]: [MCS]       Linear Dev:  0.02
[INFO] [1727103660.142530]: [MCS]       Angular Dev: 0.1
[INFO] [1727103660.143791]: [MCS]       ------
```

If your robot did not reach the goal pose, the service will return 'False' and the main node will print your error:

```
[INFO] [1727103673.018535]: [MCS]       Trying to confirm reaching the goal pose [goal]..
[ERROR] [1727103673.020986]: [MCS]      The angular goal deviation [1.6700000000000004] is too large!
```

## 7.5 Setting your task execution status to finished

Once your robot has reached all goals (therefore finishing his task),
you are supposed to tell the server by executing the following service:

```
your_user@your_machine:~$ rosservice call /master_control/set_finished "{}" 
success: True
```

This triggers stopping the time and printing your statistics if you reached all goals:

```
[INFO] [1727103684.741420]: [MCS]       Received a finished signal. Checking goals
[INFO] [1727103684.743319]: [MCS]       ------
[INFO] [1727103684.744438]: [MCS]       Goal Name:   start
[INFO] [1727103684.745496]: [MCS]       reached:     True
[INFO] [1727103684.746802]: [MCS]       time in s:   9.901
[INFO] [1727103684.747765]: [MCS]       Linear Dev:  0.02
[INFO] [1727103684.748923]: [MCS]       Angular Dev: 0.1
[INFO] [1727103684.750384]: [MCS]       ------
[INFO] [1727103684.751503]: [MCS]       ------
[INFO] [1727103684.752592]: [MCS]       Goal Name:   goal
[INFO] [1727103684.753794]: [MCS]       reached:     True
[INFO] [1727103684.754973]: [MCS]       time in s:   30.896
[INFO] [1727103684.756028]: [MCS]       Linear Dev:  0.02
[INFO] [1727103684.757647]: [MCS]       Angular Dev: 0.03
[INFO] [1727103684.760067]: [MCS]       ------
[INFO] [1727103684.763425]: [MCS]       Robot [test_robot] Reached all goals!
[INFO] [1727103684.765509]: [MCS]       Total Time: 34.53
[INFO] [1727103684.767484]: [MCS]       Linear Deviation - Total [0.04] - Median [0.02]
[INFO] [1727103684.769005]: [MCS]       Angular Deviation - Total [0.13] - Median [0.065]
```


## 7.6 Mockup Example Run

The following output of the master control server would be produced when running through the mockup task,
with some initial troubles aligning with the goal orientation:

```
..
[INFO] [1727103559.011682]: [MCS]       Reading files from folder: /home/marco/mrc_ws/src/mrc_n_codebase/master_control_tasks
[INFO] [1727103559.013987]: [MCS]       Reading File from /home/marco/mrc_ws/src/mrc_n_codebase/master_control_tasks/mockup_test.txt
[INFO] [1727103559.015688]: [MCS]       Successfully read tasks from 1 files and found the target task [mockup_test]
[INFO] [1727103643.800978]: [MCS]       Started..
[INFO] [1727103646.320201]: [MCS]       Robot [GROUP_A] has connected!
[INFO] [1727103650.233422]: [MCS]       Returning Task [mockup_test] including [2] goals
[INFO] [1727103660.134773]: [MCS]       Trying to confirm reaching the goal pose [start]..
[INFO] [1727103660.136486]: [MCS]       Successfully reached the goal pose [start]!
[INFO] [1727103660.137412]: [MCS]       ------
[INFO] [1727103660.138331]: [MCS]       Goal Name:   start
[INFO] [1727103660.139200]: [MCS]       reached:     True
[INFO] [1727103660.140409]: [MCS]       time in s:   9.901
[INFO] [1727103660.141522]: [MCS]       Linear Dev:  0.02
[INFO] [1727103660.142530]: [MCS]       Angular Dev: 0.1
[INFO] [1727103660.143791]: [MCS]       ------
[INFO] [1727103673.018535]: [MCS]       Trying to confirm reaching the goal pose [goal]..
[ERROR] [1727103673.020986]: [MCS]      The angular goal deviation [1.6700000000000004] is too large!
[INFO] [1727103681.128977]: [MCS]       Trying to confirm reaching the goal pose [goal]..
[INFO] [1727103681.131180]: [MCS]       Successfully reached the goal pose [goal]!
[INFO] [1727103681.132690]: [MCS]       ------
[INFO] [1727103681.133888]: [MCS]       Goal Name:   goal
[INFO] [1727103681.135098]: [MCS]       reached:     True
[INFO] [1727103681.136310]: [MCS]       time in s:   30.896
[INFO] [1727103681.137337]: [MCS]       Linear Dev:  0.02
[INFO] [1727103681.138695]: [MCS]       Angular Dev: 0.03
[INFO] [1727103681.139889]: [MCS]       ------
[INFO] [1727103684.741420]: [MCS]       Received a finished signal. Checking goals
[INFO] [1727103684.743319]: [MCS]       ------
[INFO] [1727103684.744438]: [MCS]       Goal Name:   start
[INFO] [1727103684.745496]: [MCS]       reached:     True
[INFO] [1727103684.746802]: [MCS]       time in s:   9.901
[INFO] [1727103684.747765]: [MCS]       Linear Dev:  0.02
[INFO] [1727103684.748923]: [MCS]       Angular Dev: 0.1
[INFO] [1727103684.750384]: [MCS]       ------
[INFO] [1727103684.751503]: [MCS]       ------
[INFO] [1727103684.752592]: [MCS]       Goal Name:   goal
[INFO] [1727103684.753794]: [MCS]       reached:     True
[INFO] [1727103684.754973]: [MCS]       time in s:   30.896
[INFO] [1727103684.756028]: [MCS]       Linear Dev:  0.02
[INFO] [1727103684.757647]: [MCS]       Angular Dev: 0.03
[INFO] [1727103684.760067]: [MCS]       ------
[INFO] [1727103684.763425]: [MCS]       Robot [GROUP_A] Reached all goals!
[INFO] [1727103684.765509]: [MCS]       Total Time: 34.53
[INFO] [1727103684.767484]: [MCS]       Linear Deviation - Total [0.04] - Median [0.02]
[INFO] [1727103684.769005]: [MCS]       Angular Deviation - Total [0.13] - Median [0.065]
```

## 7.7 Example Statemachine 

This is an example of how your statemachine should and could look like:

![](https://github.com/autonohm/mcr_n_codebase/blob/main/uml/statemachine_example.png)



