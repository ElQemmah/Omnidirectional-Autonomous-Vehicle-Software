#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import actionlib
from asv.msg import CGAction, CGGoal, CGResult, CGFeedback

def feedback_callback(feedback):
    print(feedback)

rospy.init_node('CG_action_client_node')
#topic_prefix = rospy.get_param('~topic_prefix')
client = actionlib.SimpleActionClient('asv1/vehicle_cg', CGAction)
#client2 = actionlib.SimpleActionClient('asv2/vehicle_cg', CGAction)

client.wait_for_server()
#client2.wait_for_server()
goal = CGGoal()
goal.enable = True

client.send_goal(goal)
#client2.send_goal(goal)
#print(client.get_state())
client.wait_for_result()
#client2.wait_for_result()
#print(client.get_result())
