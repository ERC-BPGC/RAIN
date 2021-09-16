#!/usr/bin/env python3	
import actionlib
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from controller import Controller

from rain_interface.msg import ExecPathAction, ExecPathResult, ExecPathFeedback

class ExecPathActionServer:

    def __init__(self):
        print("Initialized ExecPathActionServer")
        # initialize server
        self.exec_path_action_server = actionlib.SimpleActionServer("exec_path", ExecPathAction, execute_cb=self.path_request_callback, auto_start=False)
        # Advertising path subscriber
        self.odom_topic = '/odom'
        self.path_topic = '/path'
        self.path_subscriber = rospy.Subscriber(self.path_topic, Path, callback=self.path_callback)
        
        # initializing controller
        self.target_path = Path()
        self.target_path.poses.append(PoseStamped())
        goal = PoseStamped()                # for testing
        goal.pose.position.x = 1.0
        goal.pose.position.y = 2.0
        # self.target_path.poses.append(goal)
        
        self.controller = Controller(self.target_path, odom_topic=self.odom_topic)
        
        # self.action_server = actionlib.SimpleActionServer("exec_path", ExecPathAction, execute_cb=self.path_request_callback(self.path), auto_start=False)
        self.exec_path_action_server.start()

    def path_callback(self, path):
        # Stores path in global variable
        self.target_path = path
        
    def path_request_callback(self, goal):
        # Server request callback
        self.final_goal_pose_index = len(self.target_path.poses) - 1
        self.current_pose_index = 0
        
        while(self.current_pose_index < self.final_goal_pose_index):
            # initializes feedback for client
            feedback = ExecPathFeedback()
            feedback.target_pose = self.target_path.poses[self.current_pose_index+1]
            feedback.velocity = self.controller.get_velocity(feedback.target_pose)
            feedback.current_status = String("OKStatus")  # For testing
            
            self.exec_path_action_server.publish_feedback(feedback)

            if((self.controller.reached_intermediate_goal) and (self.current_pose_index < self.final_goal_pose_index)):
                self.current_pose_index += 1

        result = ExecPathResult()
        result.global_status = String("Success")    
        self.exec_path_action_server.set_succeeded(result)