#!/usr/bin/env python3

"""Use this file to get all the results. This file subscribes to odometry and odometry filtered.
    Stores some results of pose and angles as plot and also calculates some metrics
    Refer to : https://rpg.ifi.uzh.ch/docs/IROS18_Zhang.pdf

    For now, we calculate only Root Mean Square Error for all the parameters for a given time

"""
import rospy
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion as efq, quaternion_from_euler as qfe

import numpy as np
import matplotlib.pyplot as plt

class TrajectoryEvaluation:
    def __init__(self):
        rospy.Subscriber("/odom", Odometry, self.__odom_callback)
        rospy.Subscriber("/odomtery_filtered", Odometry, self.__filtered_odom_callback)

        self.ground_truth = {
            "x": [],
            "y": [],
            "theta": [],
            "t": []
        }

        self.filtered = {
            "x": [],
            "y": [],
            "theta": [],
            "t": []
        }

    def __odom_callback(self, msg: Odometry):
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

        t = msg.header.stamp.to_time()

        self.ground_truth["x"].append(x)
        self.ground_truth["y"].append(y)
        self.ground_truth["theta"].append(theta)
        self.ground_truth["t"].append(t)

    def __filtered_odom_callback(self, msg: Odometry):
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

        t = msg.header.stamp.to_time()

        self.filtered["x"].append(x)
        self.filtered["y"].append(y)
        self.filtered["theta"].append(theta)
        self.filtered["t"].append(t)

    def results(self):
        """This function calculates the errors and plots them as a function of time"""

        X, Y, Theta = np.array(self.ground_truth["x"]), np.array(self.ground_truth["y"]), np.array(self.ground_truth["theta"])
        x, y, theta = np.array(self.filtered["x"]), np.array(self.filtered["y"]), np.array(self.filtered["theta"])

        
        fig1, ax1 = plt.subplots(1, 1)

        ax1.plot(self.ground_truth["t"], Theta, label="Ground Truth")
        ax1.plot(self.filtered["t"],theta, label="Filtered")

        plt.legend()

    
        # Figure 2: Ground thruth vs Filtered Position
        fig2, ax = plt.subplots(1, 1)

        ax.plot(X, Y, label="Ground Truth")
        ax.plot(x, y, label="Filtered")


        plt.legend()

        plt.show()

if __name__ == "__main__":
    rospy.init_node("trajectory_evaluation")

    logger = TrajectoryEvaluation()

    rospy.spin()

    logger.results()

    

    


