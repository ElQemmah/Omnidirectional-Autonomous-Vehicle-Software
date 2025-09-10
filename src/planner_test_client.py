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
from asv.msg import CGAction, CGGoal, CGResult, CGFeedback, Constraint, ControllerAction, ControllerGoal, ControllerResult, ControllerFeedback 



class Planner_experiment:
    ''' Init node '''
    def __init__(self):
        # INIT NODE
        rospy.init_node('planner_experiment')


        self.client_control = actionlib.SimpleActionClient('/asv1/vehicle_controller', ControllerAction)


        self.goal_cntr = ControllerGoal()

        self.LocalConstraint1 = Constraint()
        self.LocalConstraint1.type_cnstr = "speed"
        self.LocalConstraint1.id = 1
        self.LocalConstraint1.data = [0.3, 0.3]


        self.client = actionlib.SimpleActionClient('/asv1/vehicle_cg', CGAction)

        self.goal = CGGoal()

        self.pub_request_vehicle1 = rospy.Publisher('/asv1/cg/requestC', Constraint, queue_size=10)


    def loop(self):
        rospy.sleep(1.0)

        self.goal_cntr.enable = True
        self.client_control.send_goal(self.goal)
        rospy.loginfo("Controller ENABLED")

        rospy.sleep(0.5)

        self.pub_request_vehicle1.publish(self.LocalConstraint1)
        
        rospy.loginfo("Local constraint SENT")

        rospy.sleep(0.5)

        self.goal.enable = True
        self.client.send_goal(self.goal)

        rospy.loginfo("Reference Governor ENABLED")
        
        self.client.wait_for_result()
        
        return 






if __name__ == '__main__':

    simu_plan = Planner_experiment()
    simu_plan.loop()
    


