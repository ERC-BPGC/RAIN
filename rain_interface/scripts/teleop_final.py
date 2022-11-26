#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

import math
from math import sin as s, cos as c

import numpy as np

#From Omniwheel Paper
#       Front
# rf w1/v1 v0\w0 lf
# rb w2\v2 v3/w3 lb
#        Back
p = math.pi/4
class Teleop:
    def __init__(self):
        self.wheel_vel_pub = rospy.Publisher("wheel_vel_write", Twist, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.__vel_callback)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.__pub_vel)

        self.w1 = 0
        self.w2 = 0
        self.w3 = 0
        self.w4 = 0

        # Hyper parameters
        self.r = 0.05
        self.R = 0.3

        self.theta = 0

        self.vx = 0
        self.vy = 0
        self.omega = 0
        self.v = 0
        self.vn = 0

        rospy.loginfo("Teleop Initialised")

    def transform(self):
        
        r = self.r
        R = self.R

        mat = np.array(
            [
                [-s(self.theta+p), c(self.theta + p), R],
                [-s(self.theta+ 3*p), c(self.theta + 3*p), R],
                [-s(self.theta + 5*p), c(self.theta + 5*p), R],
                [-s(self.theta + 7*p), c(self.theta + 7*p), R]
            ]
        )
        mat1 = np.array(
            [
               [0,1,R],
               [-1,0,R],
               [0,-1,R],
               [1,0,R]
            ]
        )
        # vel = np.array([self.vx, self.vy, self.omega]).reshape(-1, 1)
        # wheel_vel = (1/r)*mat @ vel
        # wheel_vel = wheel_vel.reshape(4, )
        vel = np.array([self.v, self.vn, self.omega]).reshape(-1, 1)
        wheel_vel = (1/r)*mat1 @ vel
        wheel_vel = wheel_vel.reshape(4, )

#From Omniwheel Paper
#       Front
# rf w1/v1 v0\w0 lf
# rb w2\v2 v3/w3 lb
#        Back

        self.w0 = wheel_vel[0] 
        self.w1 = wheel_vel[1] 
        self.w2 = wheel_vel[2] 
        self.w3 = wheel_vel[3] 

        print("Vel: ", self.vx, self.vy, self.omega)
        print("---")
        print("Wheel Vel: lf, rf, rb, lb", self.w0/0.10472, self.w1/0.10472, self.w2/0.10472, self.w3/0.10472)

    def __vel_callback(self, vel):
        self.vx = vel.linear.x 
        self.vy = vel.linear.y
        self.omega = vel.angular.z
        # vx = v*c(p) - vn*s(p)
        # vy = v*s(p) + vn*c(p)
        # self.v = (1/math.sqrt(2))*(-self.vx + self.vy) 
        # self.vn = (1/math.sqrt(2))*(-self.vx - self.vy)  
        self.v = (1/math.sqrt(2))*(self.vx + self.vy) 
        self.vn = (1/math.sqrt(2))*(self.vx - self.vy)  


    def __pub_vel(self, event):
        self.transform()
        vel = Twist()
        vel.linear.x = self.w0/0.10472      #lf
        vel.linear.y = self.w1/0.10472      #rf     
        vel.linear.z = self.w2/0.10472      #rb
        vel.angular.x = self.w3/0.10472     #lb
        

        self.wheel_vel_pub.publish(vel)

if __name__ == "__main__":
    rospy.init_node("teleop_node")

    teleop = Teleop()

    rospy.spin()

        
