#!/usr/bin/env python3	
from re import M
from xml.etree.ElementTree import PI

from matplotlib import test
import rospy
import numpy as np
import matplotlib.pyplot as plt
import math
from numpy import sin as s
from numpy import cos as c
from numpy import tan as t
from geometry_msgs.msg import Twist, TwistWithCovariance,TwistWithCovarianceStamped
from nav_msgs.msg import Odometry

R = 0.3
r = 0.05
p = math.pi/4
theta = 0
robot_vel = TwistWithCovarianceStamped()
wheel_odom = Odometry()
global id
id=0
def vel_read_callback(vel):
    global id
    id += 1
    w = np.array([vel.linear.x * 0.10472, vel.linear.y * 0.10472, vel.linear.z * 0.10472, vel.angular.x * 0.10472])
    print(round(vel.linear.x ,4), round(vel.linear.y,4 ),round(vel.linear.z ,4),round(vel.angular.x,4))
    wheel_vel = w.reshape(4,1)
    mat = np.array(
            [
                [-s(theta+p), c(theta + p), R],
                [-s(theta+ 3*p), c(theta + 3*p), R],
                [-s(theta + 5*p), c(theta + 5*p), R],
                [-s(theta + 7*p), c(theta + 7*p), R]
            ]
        )

#From Omniwheel Paper
#       Front
# rf w1/v1 v0\w0 lf
# rb w2\v2 v3/w3 lb
#        Back

    v = 0.5*(w[3]-w[1])
    vn = 0.5*(w[0]-w[2])

# V and Vn are shifted 45 degrees from vx and vy
    vy = v*c(p) - vn*s(p)
    vx = v*s(p) + vn*c(p)
    robot_vel.header.stamp = rospy.Time.now()
    robot_vel.header.frame_id = "camera_link"
    robot_vel.header.seq = id
    omega = (w[0]+w[1]+w[2]+w[3])/(4*R)
    robot_vel.twist.twist.linear.x = round(vx/20,1)
    robot_vel.twist.twist.linear.y = round(vy/20,1)
    robot_vel.twist.twist.angular.z = round(omega/20,1)
    wheel_odom.twist.twist = robot_vel
    robot_vel.twist.covariance = [0.1,0,0,0,0,0,
                                   0,0.1,0,0,0,0,
                                   0,0,999,0,0,0,
                                   0,0,0,999,0,0,
                                   0,0,0,0,999,0,
                                   0,0,0,0,0,0.1]
    #wheel_odom.twist.covariance ?
    #print("Vx, Vy, Omega: ", round(vx/20,1),round(vy/20,1),round(omega/20,1))
    pub.publish(robot_vel)

   
if __name__ == "__main__":
    rospy.init_node("read_vel")
    sub = rospy.Subscriber("/wheel_vel_read",Twist,vel_read_callback,queue_size=10)
    pub = rospy.Publisher("/wheel_odom",TwistWithCovarianceStamped,queue_size=10)
    
    rospy.spin()
