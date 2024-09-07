#!/usr/bin/env python

import math
import os
import copy

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Transform
from tf import transformations

from mrc_n_codebase.msg import mcs_goal_pose

from mrc_n_codebase.srv import mcs_connect

from mrc_n_codebase.srv import mcs_connectRequest
from mrc_n_codebase.srv import mcs_connectResponse

from mrc_n_codebase.srv import mcs_get_tasks
from mrc_n_codebase.srv import mcs_get_tasksRequest
from mrc_n_codebase.srv import mcs_get_tasksResponse



# Master Control Server = [MCS]
logger_prefix = "[MCS]\t"
default_frame_id = "map"

def getTfRotation(tf_quaternion):
    result_quat = Quaternion()
    result_quat.x = tf_quaternion[0]
    result_quat.y = tf_quaternion[1]
    result_quat.z = tf_quaternion[2]
    result_quat.w = tf_quaternion[3]
    return result_quat

class GoalPose:
    def __init__(self):
        self.name = "empty_goal_pose"
        self.goal_pose = Transform()

class Task:
    def __init__(self):
        self.name = "empty_task"
        self.goals = []


def read_task_from_file(path_to_file):
    rospy.loginfo(logger_prefix + 'Reading File from ' + str(path_to_file))

    file_task = Task()

    # open the provided file
    with open(path_to_file, 'r') as reader:
        line_index = 1
        # read line by line
        for line in reader:
            
            if line_index == 1:
                currentline = line.split(': ')
                currentline = currentline[1].split('\n')
                file_task.name = currentline[0]
                
            else:
                # Format: SEQ , X , Y , Yaw, Name

                # seperate the line elements by commas
                currentline = line.split(',')  

                file_goal_pose = GoalPose()

                ## translation
                file_goal_pose.goal_pose.translation.x = float(currentline[1])
                file_goal_pose.goal_pose.translation.y = float(currentline[2])
                
                ## orientation
                yaw = float(currentline[3])
                file_goal_pose.goal_pose.rotation = getTfRotation(transformations.quaternion_from_euler(0, 0, yaw))

                ## name
                file_goal_pose.name = currentline[3]
                
                # save pose in path
                file_task.goals.append(file_goal_pose)
                
            line_index += 1

    
    return file_task

    
class Master_Control_Server:
    def __init__(self):
        
        try:
            self.folder = rospy.get_param("~folder_path")
        except KeyError:
            rospy.logerr(logger_prefix + " parameter ~folder_path is required")
            exit(1)

        self.target_task_name = rospy.get_param("~target_task_name", "test1")

        # all tasks in the folder get saved here
        self.tasks_list = []
        self.target_task = Task()
        found_target_task = False

        rospy.loginfo(logger_prefix + "Reading files from folder: " + self.folder)
        files = os.listdir(self.folder)
        for f in files:
            next_elem = Task()
            next_elem = read_task_from_file(self.folder + '/' + f)                
            self.tasks_list.append(next_elem)
            if(next_elem.name == self.target_task_name):
                self.target_task = copy.deepcopy(next_elem)
                found_target_task = True  

        if(not found_target_task):
            rospy.logerr(logger_prefix + "Did not find the target task! [" + self.target_task_name + "]")
            exit(1)

        rospy.loginfo(logger_prefix + "Successfully read tasks from " + str( len(files) ) + \
                         " files and found the target [" + self.target_task_name + "]")
        
        ## setup TF publisher? --> must be timer cb


        ## Connection service + data
        self.topic_connect = rospy.get_param("~topic_connect", "master_control/connect")
        self.srv_connect = rospy.Service(self.topic_connect, mcs_connect, self.service_connect)
        
        self.robot_name = "Ohm_Robot"
        self.robot_has_connected = False

        ## Get Tasks service
        self.topic_get_tasks = rospy.get_param("~topic_get_tasks", "master_control/get_tasks")
        self.srv_get_tasks = rospy.Service(self.topic_get_tasks, mcs_get_tasks, self.service_get_tasks)

        rospy.loginfo(logger_prefix + "Started..")     

    def service_connect(self, req):
        self.robot_name = req.robot_name
        self.robot_has_connected = True
        
        # maybe turn on the kobuki leds?

        res = mcs_connectResponse()
        res.success = True

        rospy.loginfo(logger_prefix + "Robot [" + self.robot_name + "] has connected!")     
        return res
    
    def service_get_tasks(self, req):
        valid_request = True
        if(not self.robot_has_connected):
            valid_request = False
        if(req.robot_name != self.robot_name):
            valid_request = False

        res = mcs_get_tasksResponse()
        res.success = valid_request


        if (not valid_request):
            rospy.logerr(logger_prefix + "This robot [" + req.robot_name + "] has not connected before asking for tasks!")  
        else:
            res.goals = self.target_task.goals
            res.success = True
            rospy.loginfo(logger_prefix + "Returning Task [" + self.target_task.name \
                            + "] including [" + str(len(self.target_task.goals)) + "] goals")     


        # maybe turn on the kobuki leds?

        return res

rospy.init_node("master_control_node")

mcs = Master_Control_Server()

rospy.spin()
