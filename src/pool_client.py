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



class Pool_experiment:
    ''' Init node '''
    def __init__(self):
        # INIT NODE
        rospy.init_node('pool_client')

        # Controller enable for each vehicle
        self.client_control = actionlib.SimpleActionClient('/asv1/vehicle_controller', ControllerAction)
        self.client_control2 = actionlib.SimpleActionClient('/asv2/vehicle_controller', ControllerAction)
        self.client_control3 = actionlib.SimpleActionClient('/asv3/vehicle_controller', ControllerAction)

        self.goal_cntr = ControllerGoal()


        # Local Constraint building 
        self.LocalConstraint1 = Constraint()
        self.LocalConstraint1.type_cnstr = "speed"
        self.LocalConstraint1.id = 1
        self.LocalConstraint1.data = [0.3, 0.3]

        self.LocalConstraint2 = Constraint()
        self.LocalConstraint2.type_cnstr = "speed"
        self.LocalConstraint2.id = 2
        self.LocalConstraint2.data = [0.3, 0.3]

        self.LocalConstraint3 = Constraint()
        self.LocalConstraint3.type_cnstr = "speed"
        self.LocalConstraint3.id = 3
        self.LocalConstraint3.data = [0.3, 0.3]

        # Anticollision Constraint building 

        self.Anticollision_Constraint12 = Constraint()
        self.Anticollision_Constraint12.type_cnstr = "anticollision"
        self.Anticollision_Constraint12.id = 2 
        self.Anticollision_Constraint12.data = [1]
        self.Anticollision_Constraint13 = Constraint()
        self.Anticollision_Constraint13.type_cnstr = "anticollision"
        self.Anticollision_Constraint13.id = 3 
        self.Anticollision_Constraint13.data = [1]

        self.Anticollision_Constraint21 = Constraint()
        self.Anticollision_Constraint21.type_cnstr = "anticollision"
        self.Anticollision_Constraint21.id = 1 
        self.Anticollision_Constraint21.data = [1]
        self.Anticollision_Constraint23 = Constraint()
        self.Anticollision_Constraint23.type_cnstr = "anticollision"
        self.Anticollision_Constraint23.id = 3 
        self.Anticollision_Constraint23.data = [1]

        self.Anticollision_Constraint31 = Constraint()
        self.Anticollision_Constraint31.type_cnstr = "anticollision"
        self.Anticollision_Constraint31.id = 1 
        self.Anticollision_Constraint31.data = [1]
        self.Anticollision_Constraint32 = Constraint()
        self.Anticollision_Constraint32.type_cnstr = "anticollision"
        self.Anticollision_Constraint32.id = 2 
        self.Anticollision_Constraint32.data = [1]

        

        self.client = actionlib.SimpleActionClient('/asv1/vehicle_cg', CGAction)
        self.client2 = actionlib.SimpleActionClient('/asv2/vehicle_cg', CGAction)
        self.client3 = actionlib.SimpleActionClient('/asv3/vehicle_cg', CGAction)

        self.goal = CGGoal()

        self.pub_request_vehicle1 = rospy.Publisher('/asv1/cg/requestC', Constraint, queue_size=10)
        self.pub_request_vehicle2 = rospy.Publisher('/asv2/cg/requestC', Constraint, queue_size=10)
        self.pub_request_vehicle3 = rospy.Publisher('/asv3/cg/requestC', Constraint, queue_size=10)


    def loop(self):
        rospy.sleep(1.0)

        self.goal_cntr.enable = True
        self.client_control.send_goal(self.goal)
        self.client_control2.send_goal(self.goal)
        self.client_control3.send_goal(self.goal)

        rospy.loginfo("Controller ENABLED")

        rospy.sleep(0.5)

        self.pub_request_vehicle1.publish(self.LocalConstraint1)
        self.pub_request_vehicle2.publish(self.LocalConstraint2)
        self.pub_request_vehicle3.publish(self.LocalConstraint3)

        rospy.loginfo("Local constraint SENT")

        rospy.sleep(0.1)
        
        self.pub_request_vehicle1.publish(self.Anticollision_Constraint12)
        self.pub_request_vehicle1.publish(self.Anticollision_Constraint13)

        self.pub_request_vehicle2.publish(self.Anticollision_Constraint21)
        self.pub_request_vehicle2.publish(self.Anticollision_Constraint23)

        self.pub_request_vehicle3.publish(self.Anticollision_Constraint31)
        self.pub_request_vehicle3.publish(self.Anticollision_Constraint32)

        rospy.loginfo("Anticollision constraint SENT")

        rospy.sleep(0.5)

        self.goal.enable = True
        self.client.send_goal(self.goal)
        self.client2.send_goal(self.goal)
        self.client3.send_goal(self.goal)

        rospy.loginfo("Reference Governor ENABLED")
        
        self.client3.wait_for_result()
        
        return 






if __name__ == '__main__':

    simu_pool = Pool_experiment()
    simu_pool.loop()
    


