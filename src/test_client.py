#  GNU License (C) 2025 LaSa, DIMES, Univerity of Calabria

# This program is developed for LaSa, DIMES, Univerity of Calabria. 
# Its copy, use, redistribution or modification is prohibited, or requires
# you to ask for permission. All authorized modifications made to 
# the software are subject to the same conditions as the original software.
# This program is provided as is: WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# For a copy of the complete license please contact elqemmah.ay@dimes.unical.it.

# Minimal notes added; program logic unchanged.

#! /usr/bin/env python3
import rospy
import time
import actionlib
from asv.msg import ControllerAction, ControllerGoal, ControllerResult, ControllerFeedback

def feedback_callback(feedback):
    print(feedback)

rospy.init_node('controller_action_client_node')
#topic_prefix = rospy.get_param('~topic_prefix')
client = actionlib.SimpleActionClient('asv3/vehicle_controller', ControllerAction)
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
