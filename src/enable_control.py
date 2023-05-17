#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import actionlib
from asv.msg import ControllerAction, ControllerGoal, ControllerResult, ControllerFeedback

def feedback_callback(feedback):
    print(feedback)

rospy.init_node('controller_action_client_node')
#topic_prefix = rospy.get_param('~topic_prefix')
client = actionlib.SimpleActionClient('asv1/vehicle_controller', ControllerAction)
##client2 = actionlib.SimpleActionClient('asv2/vehicle_controller', ControllerAction)

client.wait_for_server()

#client2.wait_for_server()
goal = ControllerGoal()
goal.enable = True
client.send_goal(goal, feedback_cb=feedback_callback)
#client.wait_for_result()
#client2.send_goal(goal, feedback_cb=feedback_callback)
client.wait_for_result()
#print(client.get_result())
