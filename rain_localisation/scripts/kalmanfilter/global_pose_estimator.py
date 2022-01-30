#!/usr/bin/env python3

"""
This file should contain the class that reads the Lidar Data and then publishes the
change in global pose every one second after processing scans from LiDAR data

For now, I am just subscribing to Ground truth and adding some noise here to mimic the pose
we can get from LiDAR.

Function conventions
1. All callbacks are written as __(subscriber/timer name)
"""


import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from tf.transformations import euler_from_quaternion as efq

import numpy as np

class GlobalPoseExtractor:
    def __init__(self, global_start=[0, 0, 0], pose_topic="global_pose"):
        self.global_start = global_start
        self.pose_topic = pose_topic

        # self.scan_sub = rospy.Subscriber("scan", LaserScan, callback=self.__estimate_pose)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.__odom_sub)
        self.pose_pub = rospy.Publisher(self.pose_topic, Vector3, queue_size=1)

        # Timer to publish the pose topic
        self.pose_timer = rospy.Timer(rospy.Duration(1/7.0), callback=self.__pose_pub)

        self.position_vector = Vector3(*self.global_start)

    def __odom_sub(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        theta = efq(
            [
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ]
        )[2]

        x += np.random.randn() * 0.05
        y += np.random.randn() * 0.05
        theta += np.random.randn() * 0.05

        self.position_vector = Vector3(x, y, theta)

    def __pose_pub(self, event):
        self.pose_pub.publish(self.position_vector)

if __name__ == "__main__":
    rospy.init_node("global_pose_extractor")

    gpe = GlobalPoseExtractor()

    rospy.spin()
