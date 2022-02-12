#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist
from controller_copy import Controller

class Test():
    def __init__(self):
        self.odom_topic = "/odom"

        self.target_path = Path()
        self.target_path.poses.append(PoseStamped())
        self.goal = PoseStamped()                # for testing
        self.goal.pose.position.x = 3.0
        self.goal.pose.position.y = 2.0
        # self.target_path.poses.append(goal)
        
        self.controller = Controller(self.target_path, odom_topic=self.odom_topic)
        
if __name__ == "__main__":
    rospy.init_node("test")
    test = Test()
    velocity = Twist()
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    while not rospy.is_shutdown():
        velocity = test.controller.get_velocity(test.goal)
        pub.publish(velocity)
        if test.controller.reached_intermediate_goal():
            velocity.linear.x = 0.0
            velocity.linear.y = 0.0
            pub.publish(velocity)
            print("goal reached")
            break
        else:
            continue