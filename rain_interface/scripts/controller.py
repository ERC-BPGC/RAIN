#!/usr/bin/env python3	

from codecs import strict_errors
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from math import sqrt, atan2, sin, cos

class Controller():
    
    def __init__(self, path, odom_topic):
        # initialize variables
        self.path = path
        self.current_pose = PoseStamped()
        self.desired_pose = PoseStamped()
        
        self.odom_subscriber = rospy.Subscriber(odom_topic, Odometry, callback=self.odom_callback)

        self.e = 0.0
        self.e_i = 0.0
        self.e_d = 0.0
        self.e_prev = 0.0

        self.kP = 0.3
        self.kI = 0.00000025
        self.kD = 0.0

        self.max_vel = 0.5
        self.min_vel = 0.05
        self.tolerance = 0.1
        
        # rospy.spin()
        # self.error = 1000     # Define in controller/map from imported controller 

    def get_velocity(self, desired_pose: PoseStamped):
        self.desired_pose = desired_pose
        velocity = Twist()
        desired_x = self.desired_pose.pose.position.x
        desired_y = self.desired_pose.pose.position.y
        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y

        self.e = self.error_calculation()
        self.e_i += self.e
        self.e_d = self.e - self.e_prev
        desired_vel = (self.kP*self.e + self.kI*self.e_i + self.kD*self.e_d)

        self.e_prev = self.e

        if desired_vel > self.max_vel:
            desired_vel = self.max_vel
        elif desired_vel < self.min_vel:
            desired_vel = self.min_vel

        slope = atan2(desired_y-curr_y,desired_x-curr_x)
        
        x_vel = cos(slope)*desired_vel
        y_vel = sin(slope)*desired_vel
        print(x_vel)
        velocity.linear.x = x_vel
        velocity.linear.y = y_vel

        return velocity
        
    def odom_callback(self, msg: Odometry):
        # store odometry data in class level variable
        self.current_pose.pose = msg.pose.pose

    def reached_intermediate_goal(self):
        # return true if any point in the path other than goal is reached
        error = self.error_calculation() 
        return (error < self.tolerance)

    def get_current_status(self):
        # only for testing
        return String("Status: No errors")

    def error_calculation(self):
        error = sqrt((self.desired_pose.pose.position.x - self.current_pose.pose.position.x)**2 + (self.desired_pose.pose.position.y - self.current_pose.pose.position.y)**2)
        return error