#!/usr/bin/env python3	

from codecs import strict_errors
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from math import sqrt

class Controller():
    
    def __init__(self, path, odom_topic):
        # initialize variables
        self.path = path
        self.current_pose = PoseStamped()
        self.desired_pose = PoseStamped()
        
        self.odom_subscriber = rospy.Subscriber(odom_topic, Odometry, callback=self.odom_callback)

        self.e_x = 0.0
        self.e_i_x = 0.0
        self.e_d_x = 0.0
        self.e_prev_x = 0.0

        self.e_y = 0.0
        self.e_i_y = 0.0
        self.e_d_y = 0.0
        self.e_prev_y = 0.0

        self.kP_x = 0.0099
        self.kI_x = 0.00000075
        self.kD_x = 0.5

        self.kP_y = 0.0099
        self.kI_y = 0.00000075
        self.kD_y = 0.5

        self.miny = 0.01
        self.minx = 0.01
        self.tolerance = 0.1
        self.prevent_divide_by_zero = 0.000000000000000000000001
        # rospy.spin()
        # self.error = 1000     # Define in controller/map from imported controller 

    def get_velocity(self, desired_pose: PoseStamped):
        self.desired_pose = desired_pose
        velocity = Twist()
        desired_x = self.desired_pose.pose.position.x
        desired_y = self.desired_pose.pose.position.y
        curr_x = self.current_pose.pose.position.x
        curr_y = self.current_pose.pose.position.y

        if (abs(desired_x)-abs(curr_x))<0.01:
            x_ratio = 0.0
        else:
            x_ratio = (abs(desired_x)-abs(curr_x))/(abs(desired_x)-abs(curr_x)+abs(desired_y)-abs(curr_y))
            print(x_ratio)
        if (abs(desired_y)-abs(curr_y))<0.01:
            y_ratio = 0.0
        else:
            y_ratio = (abs(desired_y)-abs(curr_y))/(abs(desired_x)-abs(curr_x)+abs(desired_y)-abs(curr_y))
        # print(y_ratio)

        if desired_x-curr_x<0:
            desired_xvel = x_ratio*0.5*(-1)#*(desired_x-curr_x)/(abs(desired_x)+abs(curr_x)+self.prevent_divide_by_zero)
        else:
            desired_xvel = x_ratio*0.5
        if desired_y-curr_y<0:
            desired_yvel = y_ratio*0.5*(-1)
        else:
            desired_yvel = y_ratio*0.5#*(desired_y-curr_y)/(abs(desired_y)+abs(curr_y)+self.prevent_divide_by_zero)
        #print("desired_xvel: {}".format(desired_xvel))

        self.e_x = self.x_error()
        self.e_i_x += self.e_x
        self.e_d_x = self.e_x - self.e_prev_x
        calc_x = (self.kP_x*self.e_x + self.kI_x*self.e_i_x + self.kD_x*self.e_d_x)
        #print("calc_x: {}".format(calc_x))
        
        self.e_y = self.y_error()
        self.e_i_y += self.e_y
        self.e_d_y = self.e_y - self.e_prev_y
        calc_y = (self.kP_y*self.e_y + self.kI_y*self.e_i_y + self.kD_y*self.e_d_y)

        if calc_x < abs(desired_xvel):
            calc_x = calc_x*desired_xvel/(abs(desired_xvel)+self.prevent_divide_by_zero)
            print("1")
        elif calc_x < self.minx:
            calc_x = self.minx*desired_xvel/(abs(desired_xvel)+self.prevent_divide_by_zero)
            print("2")
        elif calc_x > abs(desired_xvel):
            calc_x = desired_xvel
            print("3")

        if (calc_y > self.miny) and (calc_y < abs(desired_yvel)):
            calc_y = calc_y*desired_yvel/(abs(desired_yvel)+self.prevent_divide_by_zero)
        elif calc_y < self.miny:
            calc_y = self.miny*desired_yvel/(abs(desired_yvel)+self.prevent_divide_by_zero)
        elif calc_y > abs(desired_yvel):
            calc_y = desired_yvel
            
        velocity.linear.x = calc_x
        velocity.linear.y = calc_y

        self.e_prev_x = self.e_x
        self.e_prev_y = self.e_y

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

    def x_error(self):
        return abs(self.desired_pose.pose.position.x - self.current_pose.pose.position.x)

    def y_error(self):
        return abs(self.desired_pose.pose.position.y - self.current_pose.pose.position.y)