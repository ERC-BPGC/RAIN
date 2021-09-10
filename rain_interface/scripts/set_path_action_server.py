#!/usr/bin/env python3
import rospy
import actionlib
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from std_msgs.msg import String

from rain_interface.msg import SetPathResult, SetPathAction

class SimpleActionServerTrail:
    def __init__(self):
        print("Initiated SimpleActionServerTrail")
        self.goal = SetPathResult()
        self.action_server = actionlib.SimpleActionServer("set_path", SetPathAction, execute_cb=self.execute_callback, auto_start=False)

        self.action_server.start()

    def execute_callback(self, goal):
        path = Path()
        path.header.frame_id = "odom"
        path.header.stamp = rospy.Time.now()

        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position = Point(1, 2, 3)
        pose.pose.orientation = Quaternion(0, 0, 0, 1)

        for i in range(5):
            path.poses.append(pose)

        self.goal.path = path
        self.goal.status = String("PathReached")

        self.action_server.set_succeeded(self.goal)

if __name__ == "__main__":
    rospy.init_node("sas_trial")
    server = SimpleActionServerTrail()

    rospy.spin()


        