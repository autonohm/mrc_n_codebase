#!/usr/bin/env python

import math
import os
import copy

import rospy
import tf2_ros
import tf2_msgs.msg
import tf.transformations as tft
import geometry_msgs.msg 

from mrc_n_codebase.msg import mcs_goal_pose

from mrc_n_codebase.srv import mcs_connect
from mrc_n_codebase.srv import mcs_connectRequest
from mrc_n_codebase.srv import mcs_connectResponse

from mrc_n_codebase.srv import mcs_get_tasks
from mrc_n_codebase.srv import mcs_get_tasksRequest
from mrc_n_codebase.srv import mcs_get_tasksResponse

from mrc_n_codebase.srv import mcs_confirm_goal_reached
from mrc_n_codebase.srv import mcs_confirm_goal_reachedRequest
from mrc_n_codebase.srv import mcs_confirm_goal_reachedResponse

from mrc_n_codebase.srv import mcs_set_finished
from mrc_n_codebase.srv import mcs_set_finishedRequest
from mrc_n_codebase.srv import mcs_set_finishedResponse


# Master Control Server = [MCS]
logger_prefix = "[MCS]\t"
default_frame_id = "map"

def getTfRotation(tf_quaternion):
    result_quat = geometry_msgs.msg.Quaternion()
    result_quat.x = tf_quaternion[0]
    result_quat.y = tf_quaternion[1]
    result_quat.z = tf_quaternion[2]
    result_quat.w = tf_quaternion[3]
    return result_quat

def getQuatArray(tf_quaternion):
    return [tf_quaternion.x, 
            tf_quaternion.y,
            tf_quaternion.z,
            tf_quaternion.w]

class GoalPose:
    def __init__(self):
        self.name = "empty_goal_pose"
        self.goal_pose = geometry_msgs.msg.Transform()

class Task:
    def __init__(self):
        self.name = "empty_task"
        self.goals = []

class GoalScore:
    def __init__(self):
        self.name = "empty_goal_pose"
        self.is_reached = False
        self.deviation_lin = 10000.0
        self.deviation_ang = 10000.0
        self.time_to_reach_s = 10000.0

    def print(self, prefix):
        rospy.loginfo(prefix + "------")
        rospy.loginfo(prefix + "Goal Name:   " + self.name)
        rospy.loginfo(prefix + "reached:     " + str(self.is_reached))
        rospy.loginfo(prefix + "time in s:   " + str(round(self.time_to_reach_s, 3)))
        rospy.loginfo(prefix + "Linear Dev:  " + str(round(self.deviation_lin,   3)))
        rospy.loginfo(prefix + "Angular Dev: " + str(round(self.deviation_ang,   3)))
        rospy.loginfo(prefix + "------")


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
                file_goal_pose.goal_pose.translation.x = float(currentline[2])
                file_goal_pose.goal_pose.translation.y = float(currentline[3])
                
                ## orientation
                yaw = float(currentline[4])
                file_goal_pose.goal_pose.rotation = getTfRotation(tft.quaternion_from_euler(0, 0, yaw))

                ## name
                file_goal_pose.name = str(currentline[1]).replace(" ", "")
                
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
        
        # create the goal list for the task
        self.goal_scores = []
        for task in self.target_task.goals:
            score = GoalScore()
            score.name = task.name
            self.goal_scores.append(score)

        rospy.loginfo(logger_prefix + "Successfully read tasks from " + str( len(files) ) + \
                         " files and found the target task [" + self.target_task_name + "]")
        
        ## setup TF listener and broadcaster
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Timer(rospy.Duration(0.5), self.pub_poses_as_tf)

        ## Connection service + data
        self.topic_connect = rospy.get_param("~topic_connect", "master_control/connect")
        self.srv_connect = rospy.Service(self.topic_connect, mcs_connect, self.service_connect)
        
        self.robot_name = "Ohm_Robot"
        self.robot_has_connected = False

        ## Get Tasks service
        self.topic_get_tasks = rospy.get_param("~topic_get_tasks", "master_control/get_tasks")
        self.srv_get_tasks = rospy.Service(self.topic_get_tasks, mcs_get_tasks, self.service_get_tasks)

        self.task_exec_timer = rospy.Time.now()
        self.sent_tasks_to_robot = False

        ## Confirm goal reached service
        self.topic_confirm_goal_reached = rospy.get_param("~topic_confirm_goal_reached", "master_control/confirm_goal_reached")
        self.srv_confirm_goal_reached = rospy.Service(self.topic_confirm_goal_reached, mcs_confirm_goal_reached, self.service_confirm_goal_reached)

        self.robot_frame_id = rospy.get_param("~robot_frame_id", "base_link")
        self.max_goal_dev_lin = rospy.get_param("~max_goal_deviation_linear", 0.1)
        self.max_goal_dev_ang = rospy.get_param("~max_goal_deviation_angular", 0.2)


        ## set finished service
        self.topic_set_finished = rospy.get_param("~topic_set_finished", "master_control/set_finished")
        self.srv_set_finished = rospy.Service(self.topic_set_finished, mcs_set_finished, self.service_set_finished)


        rospy.loginfo(logger_prefix + "Started..")     

    def pub_poses_as_tf(self, event):
        
        for gp in self.target_task.goals:
    
            #rospy.loginfo(logger_prefix + "pub TF " + gp.name + " - tx=" + str(gp.goal_pose.translation.x) + " - ty=" + str(gp.goal_pose.translation.y) )    
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = 'map'
            t.child_frame_id = gp.name
            t.transform = gp.goal_pose

            # Publish the transform
            self.tf_broadcaster.sendTransform(t)

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
            rospy.logerr(logger_prefix + "This robot [" + req.robot_name + "] has not connected before. Rejecting task request!")  
        else:
            res.goals = self.target_task.goals
            res.success = True
            rospy.loginfo(logger_prefix + "Returning Task [" + self.target_task.name \
                            + "] including [" + str(len(self.target_task.goals)) + "] goals")     


        # maybe turn on the kobuki leds?

        # if this is the first time the task is sent to this robot, we start the exec time
        if (not self.sent_tasks_to_robot):
            self.task_exec_timer = rospy.Time.now()
        
        self.sent_tasks_to_robot = True

        return res

    def service_confirm_goal_reached(self, req):
        res = mcs_confirm_goal_reachedResponse()
        res.success = False

        rospy.loginfo(logger_prefix + "Trying to confirm reaching the goal pose [" + req.goal_name + "]..")

        if( not (self.robot_has_connected and self.sent_tasks_to_robot) ):
            rospy.logerr(logger_prefix + "You need to connect the robot and request tasks before confirming goals!")  
            return res


        found_target = False
        target = GoalPose()
        for gp in self.target_task.goals:
            if(gp.name == req.goal_name):
                found_target = True
                target = gp

        if not found_target:
            rospy.logerr(logger_prefix + "Your provided goal pose [" + req.goal_name + "] is not in the task list!")  
            return res        

        # get robot tf
        try:
            # Get the latest available transform between 'odom' and 'base_link'
            transform = self.tf_buffer.lookup_transform('map', self.robot_frame_id, rospy.Time(0), rospy.Duration(1.0))
        except tf2_ros.LookupException as e:
            rospy.logwarn(logger_prefix + "Transform lookup failed: %s", str(e))
            return res
        except tf2_ros.ConnectivityException as e:
            rospy.logwarn(logger_prefix + "Connectivity issue: %s", str(e))
            return res
        except tf2_ros.ExtrapolationException as e:
            rospy.logwarn(logger_prefix + "Extrapolation issue: %s", str(e))
            return res

        # check if its smaller 
        goal_score = GoalScore()
        diff_x = target.goal_pose.translation.x - transform.transform.translation.x
        diff_y = target.goal_pose.translation.y - transform.transform.translation.y

        (target_roll, target_pitch, target_yaw) = tft.euler_from_quaternion(getQuatArray(target.goal_pose.rotation))
        (robot_roll, robot_pitch, robot_yaw) = tft.euler_from_quaternion(getQuatArray(transform.transform.rotation))
        diff_phi = target_yaw - robot_yaw
        diff_phi = (diff_phi + math.pi) % (2 * math.pi) - math.pi
        diff_phi = abs(diff_phi)
        
        goal_score.deviation_lin = math.sqrt(diff_x * diff_x + diff_y * diff_y)
        goal_score.deviation_ang = diff_phi
        goal_score.name = req.goal_name

        if(goal_score.deviation_lin > self.max_goal_dev_lin):
            rospy.logerr(logger_prefix + "The linear goal deviation [" + str(goal_score.deviation_lin) + "] is too large!")
            return res
        elif(goal_score.deviation_ang > self.max_goal_dev_ang):
            rospy.logerr(logger_prefix + "The angular goal deviation [" + str(goal_score.deviation_ang) + "] is too large!")
            return res 

        # at this point we can confirm that the goal is reached
        goal_score.is_reached = True
        goal_score.time_to_reach_s = (rospy.Time.now() - self.task_exec_timer).to_sec()

        rospy.loginfo(logger_prefix + "Successfully reached the goal pose [" + req.goal_name + "]!")
        goal_score.print(logger_prefix)

        for i, score in enumerate(self.goal_scores):
            if score.name == goal_score.name:
                self.goal_scores[i] = goal_score

        res.success = True
        return res

    def service_set_finished(self, req):
        res = mcs_set_finishedResponse()
        res.success = False
        rospy.loginfo(logger_prefix + "Received a finished signal. Checking goals")     

        goals_reached = 0
        total_goals = len(self.goal_scores)
        total_dev_lin = 0
        total_dev_ang = 0
        for score in self.goal_scores:
            score.print(logger_prefix)
            if score.is_reached:
                goals_reached += 1
                total_dev_lin += score.deviation_lin
                total_dev_ang += score.deviation_ang

        if(goals_reached == total_goals):
            rospy.loginfo(logger_prefix + "Reached all goals!")
            total_time = (rospy.Time.now() - self.task_exec_timer).to_sec()
            rospy.loginfo(logger_prefix + "Total Time: " + str(round(total_time, 3)))
            rospy.loginfo(logger_prefix + "Linear Deviation - Total [" + str(round(total_dev_lin, 3)) + "] - Median [" + str(round(total_dev_lin/total_goals, 3)) + "]")
            rospy.loginfo(logger_prefix + "Angular Deviation - Total [" + str(round(total_dev_ang, 3)) + "] - Median [" + str(round((total_dev_ang/total_goals, 3))) + "]")
            res.success = True
        else:
            rospy.logwarn(logger_prefix + "Did NOT reach all poses! Only [" + str(goals_reached) + "] out of [" + str(total_goals) + "]")

        return res


rospy.init_node("master_control_node")

mcs = Master_Control_Server()

rospy.spin()
