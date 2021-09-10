#!/usr/bin/env python3
from actionlib import goal_id_generator
import rospy
import actionlib

from rain_interface.msg import SetPathAction, SetPathGoal
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

def request_global_path_using_SAC(goal_pose: PoseStamped)->Path:
    client = actionlib.SimpleActionClient('set_path', SetPathAction)
    client.wait_for_server()

    result = SetPathGoal()
    result.goal = goal_pose
    client.send_goal(goal=result)

    client.wait_for_result(rospy.Duration.from_sec(2))

    goal = client.get_result()

    try:
        return goal.path
    except AttributeError:
        return Path()
if __name__ == "__main__":
    rospy.init_node("sac_trial")
    
    goal = PoseStamped()
    print(request_global_path_using_SAC(goal))