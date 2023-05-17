#! /usr/bin/env python3
# -*- coding: utf-8 -*-

#  GNU License (C) 2021  ELTOS
#  This program is developed for Unical by Unical's researchers.
#  For a copy of the complete license please contact  {franco.torchiaro}{elqemmah.ay}@dimes.unical.it.

import  rospy
import  numpy              as np
from    asv.msg            import Vector, Info_msg
import threading
AVAILABLE_CMD = ['ANTI', 'INFO', 'PROX']
R_1 = 10

class Controlled_Vehicle:
    def __init__(self):
        ''' Init node '''
        # INIT NODE
        rospy.init_node('controlled_vehicle_node')

        ######### TOPIC PREFIX INFO #########
        topic_prefix = rospy.get_param('~topic_prefix')
        num = topic_prefix.split('asv') 
        self.rate = rospy.Rate(200)
        self._lock_x = threading.Lock()
        self._lock_xc = threading.Lock()
        self._lock_x.acquire()
        self._lock_xc.acquire()

        self.id = int(num[-1]) 
        ######### BUILDING LOCAL INFO FOR VEHICLEINFO TOPIC ########### 

        self.states = np.array(())  # made of x, xc = xa 
        self.xc = np.array(())
        self.g = np.zeros((3, 1)) # made of g

        self.rs = rospy.Subscriber('vehicle/state', Vector, self.state_callback)
        self.rsc = rospy.Subscriber('controller/state', Vector, self.state_controller_callback)
        rsg = rospy.Subscriber('vehicle/reference', Vector, self.state_g_callback)

        self.info_pub = rospy.Publisher('vehicle/info', Info_msg, queue_size=10)

    ############## LOCAL INFO CALLBACKS #############################
    def state_callback(self,msg):
        if(self.states.size == 0):
            tmp = msg.data
            self.states = np.zeros((6, 1))
            self.states = np.reshape(np.array((tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],tmp[5])),(6,1))
            self.g = self.states[0:3]
            self._lock_x.release()
        else:
            tmp = msg.data
            self.states[0:6] = np.reshape(np.array((tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],tmp[5])),(6,1))

    def state_controller_callback(self,msg):
        if(self.xc.size  == 0):
            tmp = msg.data
            self.xc = np.zeros((3, 1))
            self.xc = np.reshape(np.array((tmp[0],tmp[1],tmp[2])),(3,1))
            self._lock_xc.release()
        else:
            tmp = msg.data
            self.xc = np.reshape(np.array((tmp[0],tmp[1],tmp[2])),(3,1))
    
    def state_g_callback(self,msg):
        tmp = msg.data
        self.g = np.reshape(np.array((tmp[0],tmp[1],tmp[2])),(3,1))


    def loop(self):
        ''' Life cycle
        Collect all data of vehicle
        '''
        self._lock_xc.acquire()
        self._lock_x.acquire()
        while not rospy.is_shutdown():
            ######### VEHICLE INFO TOPIC PUB ################
            to_send = Info_msg()
            to_send.id = self.id
            to_send.x = self.states.flatten().tolist() + self.xc.flatten().tolist()
            to_send.g = self.g.flatten().tolist()
            self.info_pub.publish(to_send)
            self.rate.sleep()
        



if __name__ == '__main__':  
    CV = Controlled_Vehicle()
    CV.loop() 