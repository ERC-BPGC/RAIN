#!/usr/bin/env python3	

from codecs import strict_errors
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion
from nav_msgs.msg import Odometry

class Controller():
    
    def __init__(self, path, odom_topic):
        # initialize variables
        self.path = path
        self.current_pose = PoseStamped()
        self.desired_pose = PoseStamped()
        
        self.odom_subscriber = rospy.Subscriber(odom_topic, Odometry, callback=self.odom_callback)

        self.tolerance = 0.1
        self.error = 1000     # Define in controller/map from imported controller 

    def get_velocity(self, desired_pose: PoseStamped):
        self.desired_pose = desired_pose
        velocity = Twist()
        velocity.linear.x = 0.1     # only for testing
        # implement controller
        # velocity, error = controller_function()
        return velocity
        
    def odom_callback(self, msg: Odometry):
        # store odometry data in class level variable
        self.current_pose = msg.pose.pose

    def reached_intermediate_goal(self):
        # return true if any point in the path other than goal is reached 
        return (self.error < self.tolerance)

    def get_current_status(self):
        # only for testing
        return String("Status: No errors")