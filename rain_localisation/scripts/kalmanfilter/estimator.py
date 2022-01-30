#!/usr/bin/env python3

"""
This is our great estimator file. We will have two Kalman Filters running at different 
frequencies to estimate both velocity and position using IMU, Wheel Encoder, LiDAR
"""

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, PoseWithCovariance, Quaternion, Point, Pose, TwistWithCovariance
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

import numpy as np
from numpy import sin as s, cos as c
from kalman import KalmanFilter as KF

from tf.transformations import euler_from_quaternion as efq, quaternion_from_euler as qfe

class MyGreatEstimator:
    def __init__(self):
        # Pubs and subs
        # self.w3_sub = rospy.Subscriber('/omnibase/back_left_joint_velocity_controller/command', Float64, self.__w3_sub)
        # self.w4_sub = rospy.Subscriber('/omnibase/back_right_joint_velocity_controller/command', Float64, self.__w4_sub)
        # self.w2_sub = rospy.Subscriber('/omnibase/front_left_joint_velocity_controller/command', Float64, self.__w2_sub)
        # self.w1_sub = rospy.Subscriber('/omnibase/front_right_joint_velocity_controller/command', Float64, self.__w1_sub)

        self.global_pose_sub = rospy.Subscriber('global_pose', Vector3, self.__global_pose_sub)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.__imu_sub)

        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.__odom_sub)
        
        self.estimated_odom_pub = rospy.Publisher('/odomtery_filtered', Odometry, queue_size=10)

        self.w1, self.w2, self.w3, self.w4 = 0.0, 0.0, 0.0, 0.0

        self.delta_t1 = 1/981.0
        self.delta_t2 = 1/79.0

        # Kalman Filter 1
        self.A1 = np.diag([1, 1, 0])
        self.B1 = np.diag([self.delta_t1, self.delta_t1, 1])
        self.C1 = np.eye(3)

        self.Q1 = 0.001 * np.eye(3)
        self.R1 = 0.0001 * np.eye(3)

        # Kalman Filter 2
        self.A2 = np.eye(3)
        self.B2 = self.delta_t2 * np.eye(3)
        self.C2 = np.eye(3)

        self.Q2 = 0.00001 * np.eye(3)
        self.R2 = 0.001 * np.eye(3)

        self.kf1 = KF(self.A1, self.B1, self.C1)
        self.kf1.covariance_matrices(self.Q1, self.R1)

        self.kf1.initialise(np.zeros((3, 1)), 0.1 * np.eye(3))

        self.kf2 = KF(self.A2, self.B2, self.C2)
        self.kf2.covariance_matrices(self.Q2, self.R2)

        self.kf2.initialise(np.zeros((3, 1)), 0.01 * np.eye(3))

        # All required data
        self.wheel_velocity = None
        self.linear_accell = None
        self.angular_vel = None
        self.global_pose = None

        self.estimated_velocity = None
        self.estimated_global_pose = None


    def __odom_sub(self, msg):
        self.wheel_velocity = msg.twist.twist

        vel = np.array([self.wheel_velocity.linear.x, self.wheel_velocity.linear.y, self.wheel_velocity.angular.z]).reshape(3, 1)

        # Correct kf1 here
        self.kf1.correct(vel)

        self.estimated_velocity = self.kf1.x0
        
        vx, vy, w = self.estimated_velocity[0][0], self.estimated_velocity[1][0],self.estimated_velocity[2][0]

        # print(f"vx: {vx:0.2f}, vy: {vy: 0.2f}, w: {w: 0.2f}")

        # Run kf2 here
        self.kf2.update(self.kf1.x0)

    def __imu_sub(self, msg):
        self.linear_accell = msg.linear_acceleration
        self.angular_vel = msg.angular_velocity

        uk = np.array([self.linear_accell.x, self.linear_accell.y, self.angular_vel.z]).reshape(3, 1)

        # Run kf1 here
        self.kf1.update(uk)

    def __global_pose_sub(self, msg):
        self.global_pose = msg

        x, y, theta = msg.x, msg.y, msg.z

        pose = np.array([x, y, theta]).reshape(3, 1)

        # Update global pose here
        self.kf2.correct(pose)

        self.estimated_global_pose = self.kf2.x0
        
        x, y, theta = self.estimated_global_pose[0][0],self.estimated_global_pose[1][0], self.estimated_global_pose[2][0]

        print(f"x: {x:0.2f}, y: {y: 0.2f}, theta: {theta: 0.2f}")

        poseStamped = PoseWithCovariance()

        quat = qfe(0, 0, theta)
        poseStamped.pose = Pose(position=Point(x, y, 0), orientation=Quaternion(*quat))
        poseStamped.covariance = [0.0 for i in range(36)]
        
        twistStamped = TwistWithCovariance()
        twistStamped.covariance = [0.0 for i in range(36)]

        odometry_msg = Odometry()
        odometry_msg.header.frame_id="world"
        odometry_msg.header.stamp = rospy.Time.now()
        odometry_msg.child_frame_id = "estimate"
        odometry_msg.pose = poseStamped
        odometry_msg.twist = twistStamped

        self.estimated_odom_pub.publish(odometry_msg)

    # Wheel Velocity subscribers
    # Use this for subscribing to the wheel velocities directly  

    def __w1_sub(self, msg):
        self.w1 = msg.data

    def __w2_sub(self, msg):
        self.w2 = msg.data

    def __w3_sub(self, msg):
        self.w3 = msg.data

    def __w4_sub(self, msg):
        self.w4 = msg.data


    # Forward velocity of omnibase
    def C(self, theta, w_array):
        """Use this function if you are taking velocity directly from wheel encoders.
        For simulation, I am directly subscribing to ground truth and obtaining x_dot, y_dot, w.
        """
        J = np.array([
            [-s(theta+np.pi/4), c(theta + np.pi/4), 1/(2*self.R)],
            [-s(theta+3*np.pi/4), c(theta+3*np.pi/4), 1/(2*self.R)],
            [-s(theta+5*np.pi/4), c(theta+5*np.pi/4), 1/(2*self.R)],
            [-s(theta+7*np.pi/4), c(theta+7*np.pi/4), 1/(2*self.R)]
            ])
        
        return (self.r/2)*J.T @ w_array

if __name__ == "__main__":
    rospy.init_node("my_great_estimator")

    mge = MyGreatEstimator()

    rospy.spin()
    




    