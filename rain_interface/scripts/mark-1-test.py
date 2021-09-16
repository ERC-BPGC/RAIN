#!/usr/bin/env python3	

import rospy
from exec_path_action_server import ExecPathActionServer
from set_path_action_server import SimpleActionServerTrail
from rain_interface.msg import SetPathAction, ExecPathAction
from dynamic_manager import DynamicManager

if __name__ == '__main__':

    rospy.init_node('rain_dynamic_manager')
    
    set_path_server = SimpleActionServerTrail()
    exec_path_server = ExecPathActionServer()
    
    dyn_manager = DynamicManager()
    dyn_manager.send_get_path_action()
    
    rospy.spin()