import actionlib
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion, Twist
from nav_msgs.msg import Path, Odometry

from rain_interface.msg import SetPathAction, SetPathGoal, ExecPathAction, ExecPathFeedback, ExecPathGoal

class DynamicManager:
    def __init__(self, goal_topic="/goal", vel_topic="/cmd_vel", odom_topic="/odom", path_topic="/path"):
        self.goal_topic = goal_topic
        self.vel_topic = vel_topic
        self.odom_topic = odom_topic
        self.path_topic = path_topic

        # All subscribers and publishers here
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=10)
        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=10)
        self.goal_sub = rospy.Subsriber(self.goal_topic, PoseStamped, self.__goal_sub)
        self.odom_sub = rospy.Subsriber(self.odom_topic, Odometry, self.__odom_sub)

        # Initialise all action clients here
        self.set_path_client = actionlib.SimpleActionClient("set_path", SetPathAction)
        self.execute_path_client = actionlib.SimpleActionClient("exec_path", ExecPathAction)

        # Variables
        self.current_position = Point(0, 0, 0)
        self.current_orientation = Quaternion(0, 0, 0, 1)

        self.goal = None

    # Subscriber functions
    def __odom_sub(self, msg):
        self.current_position = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation

    def __goal_sub(self, msg):
        # This function will set the goal. Only published when goal has to be set
        self.goal = msg

        # call the path planning action
        self.send_get_path_action()

    # Action Clients
    def send_get_path_action(self):
        # If the server is not active even after three seconds od goal sent. Kill it
        self.set_path_client.wait_for_server(timeout=rospy.Duration(3))
        
        set_path_goal = SetPathGoal()
        set_path_goal.goal = self.goal

        self.set_path_client.send_goal(set_path_goal, done_cb=self.send_exec_path_action)

        self.set_path_client.wait_for_result()
        result = self.set_path_client.get_result()
        rospy.loginfo("Set Path Executed with status %s. Done function called to execute the path", result.status)

    def send_exec_path_action(self, result):
        # Publish the path
        self.__path_publish(result.path)

        self.execute_path_client.wait_for_server(timeout=rospy.Duration(3))
        
        # This will recieve the SetPathResult as the parameter
        # All you have to do is to send the path as a parameter
        exec_path_goal = ExecPathGoal()
        exec_path_goal.path = result.path

        self.execute_path_client.send_goal(exec_path_goal, feedback_cb=self.__cmd_vel_publish)

        self.execute_path_client.wait_for_result()

        result = self.execute_path_client.get_result()

        rospy.loginfo("Completed Executing path with status %s", result.global_status)

    # Publishing Functions
    def __path_publish(self, path):
        self.path_pub.publish(path)

    def __cmd_vel_publish(self, feedback):
        # Publish velocity commands from the feedback
        cmd_vel = feedback.velocity

        self.vel_pub.publish(cmd_vel)

