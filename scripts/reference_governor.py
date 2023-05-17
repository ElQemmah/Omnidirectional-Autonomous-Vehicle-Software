#! /usr/bin/env python3
# -*- coding: utf-8 -*-

#  Copyright (C) 2021  Vincenzo D'Angelo
#  This program is developed for Applicon srl by Vincenzo D'Angelo. 
#  Its copy, use, redistribution or modification is prohibited, or requires
#  you to ask for permission. All authorized modifications made to 
#  the software are subject to the same conditions as the original software.
#  This program is provided as is: WITHOUT ANY WARRANTY; without even the
#  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#  For a copy of the complete license please contact develop@applicon.it.

import rospy
import numpy as np
import actionlib
import sys
import os
import threading
sys.path.insert(0,os.path.expanduser('~')+'/ares_ros_ws/src/asv/lib/cg/')

import dynamic_distributed_command_governor as DCG
#import command_governor as DCG
from asv.msg import Vector, Matrix, CGAction, CGResult, Constraint, Neigh_data, Info_msg
from std_msgs.msg import Bool


class ReferenceGovernor:

    def __init__(self):
        ''' Init node '''
        # INIT NODE
        rospy.init_node('reference_governor_node')

        # PARAMETERS
        topic_prefix = rospy.get_param('~topic_prefix')
        # for subscribe actions, topic_prefix is setted in launch file
        # Info used to sync different CGs 
        num = topic_prefix.split('asv')
        iden = int(num[-1])
        self.iden = iden
        self._lock = threading.Lock()
        self.mutex = threading.Lock()
        self._lock.acquire()
        self.msg_queue = []
        self.token_enabled = True
        ############# System info #############
        self.Phi = np.array(rospy.get_param('system/Phi', (np.zeros((9*9,1))).tolist()))
        self.Phi = np.array(self.Phi).reshape(9,9)
        self.G = np.array(rospy.get_param('system/G', (np.zeros((9*3,1))).tolist()))
        self.G = np.array(self.G).reshape(9,3)
        ############# Control info ############
        self.F = np.array(rospy.get_param('control/feedback_gain', (np.zeros((3*6,1))).tolist()))
        self.F = np.array(self.F).reshape(3,6)
        self.f = np.array(rospy.get_param('control/feedforward_gain', (np.zeros((3*3,1))).tolist()))
        self.f = np.array(self.f).reshape(3,3)
        ############## CG info ###############
        Tc_cg = rospy.get_param('command_governor/Tc_cg', 0.1)
        self.rate = rospy.Rate(1/Tc_cg)
        self.Psi = np.array(rospy.get_param('command_governor/Psi', (np.zeros((3*3,1))).tolist()))
        self.Psi = np.array(self.Psi).reshape(3,3)
        self.k0 = rospy.get_param('command_governor/k0', 20)
        self.L = np.zeros((9,3))
        self.Hc = np.row_stack((np.column_stack((np.eye(6), np.zeros((6,3)))), np.column_stack((-self.F, self.f))))
        ########## SELF CG INITIALIZATION ############
        self.dcg = DCG.DynamicDistributedCommandGovernor(self.iden, self.Phi, self.G, self.Hc, self.L, self.Psi, self.k0, 'Gurobi')
        self.info_map = {}
        ############# Get initial pose ########
        xinitial = rospy.get_param('~xinitial',0.0)
        yinitial = rospy.get_param('~yinitial',0.0)
        yaw_initial = rospy.get_param('~yaw_initial',0.0) 
        #yaw_initial = float(yaw_initial) + 1.57 # Add pi/2 in order to satisfy ENU spec.
        # Initial State
        self.x_initial = None
        self.y_initial = None
        self.yaw_initial = None
        while(self.yaw_initial is None or self.x_initial is None or self.y_initial is None):
            rospy.sleep(2.5)
            msg = rospy.wait_for_message('vehicle/state', Vector)
            tmp = msg.data
            x =  np.reshape(np.array((tmp[0],tmp[1],tmp[2],tmp[3],tmp[4],tmp[5])),(6,1))
            x = np.reshape(x, (6,1))
            self.x_initial = x[0]
            self.y_initial = x[1]
            self.yaw_initial = x[2]


        # Initial State
        #self.x = np.array([-float(yinitial),float(xinitial),yaw_initial,0.0,0.0,0.0])
        self.x = np.array([self.x_initial,self.y_initial,self.yaw_initial,0.0,0.0,0.0])
        self.x = np.reshape(self.x, (6,1))
        # Initial Controller State
        self.xc = np.array(np.dot(np.dot(np.linalg.pinv(self.f),self.F),self.x))
        self.xc = np.reshape(self.xc, (3,1))
        # Initial Reference and Adm. Reference equal to initial position
        #self.r = np.array([-float(yinitial),float(xinitial),yaw_initial])
        self.r = np.array([self.x_initial,self.y_initial,self.yaw_initial])
        self.r = np.reshape(self.r, (3,1))
        self.g = self.r
        
        self.token_enabled = True

        # TOPICS
        rospy.Subscriber('token/enable', Bool, self.token_enable_callback)
        # Topic used to recieve local constraints or swarm constraints 
        rospy.Subscriber('cg/add_cnstr', Constraint, self.cg_config_callback)
        rospy.Subscriber('cg/remove_cnstr', Constraint, self.cg_config_rm_callback)
        # Topic used to pass refernce generated from Planner to CG
        rospy.Subscriber('cg/reference', Vector, self.cg_reference_callback)
        rospy.Subscriber('cost_matrix_update', Matrix, self.update_matrix_callback)
        #rospy.Subscriber('vehicleinfo', Vector, self.state_callback) #Data from coomunication manager
        rospy.Subscriber('vehicle/info', Info_msg, self.vehicle_info_callback)
        # Topic used to pass through callback the reference generated by CG 
        self.g_pub = rospy.Publisher('vehicle/reference', Vector, queue_size=1)
        self.token_release = rospy.Publisher('token/release', Bool, queue_size=1)
        # Ini. of Admissible Reference
        self.g_pub.publish((self.g.flatten()).tolist()) 
        # Topic used to get information about the neiborghs 
        rospy.Subscriber('neigh/info', Neigh_data, self.neigh_info_callback)

        # ACTION SERVER
        self.add_vehicle_constraints(speed=[0.1, 0.1])
        # Server to handle references got from Planner Module
        self.server = actionlib.SimpleActionServer('vehicle_cg', CGAction, self.cg_controller_callback, False)
        self.server.start()
        
    ############# CG CONFIG CALLBACKS ###################
    def cg_config_callback(self, msg):
        ''' Update vehicle's current reference (usually got from Planner) '''
        with self.mutex:
            self.msg_queue.append(msg)
    
    def token_enable_callback(self, msg):
        self.token_enabled = msg.data
    
    def manage_cg_config_msg(self, msg):
        if(msg.info.id in self.info_map.keys() and msg.info.id != self.iden):
            return
        constrains_type = msg.type_cnstr
        speed_value = []
        position_value = []
        thrust_value = []
        anticollision_value = -1
        proximity_value = -1
        cont = 0 
        local_cnstr = False
        for type in constrains_type:
            if(type == "speed" or type == "position" or type == "thrust"):
                if(type=="speed"):
                    speed_value = msg.data[cont:cont+2]
                elif(type=="position"):
                    position_value = msg.data[cont:cont+2]
                elif(type=="thrust"):
                    thrust_value = msg.data[cont:cont+2]
                cont += 2
                local_cnstr = True
            else:
                if(type=="anticollision"):
                    anticollision_value = msg.data[cont]
                elif(type=="proximity"):
                    proximity_value = msg.data[cont]
                cont += 1    

        if(local_cnstr):
            self.add_vehicle_constraints(position=position_value,speed=speed_value,thrust=thrust_value)
        if(anticollision_value != -1 or proximity_value != -1):
            self.add_swarm_constraints(idpassed=msg.info.id,anticollision=anticollision_value,proximity=proximity_value)
            self.info_map[msg.info.id] = msg.info
            #rospy.loginfo(msg.info)
        
            
    def neigh_info_callback(self, msg):
        for neigh in msg.info:
            if(neigh.id in self.info_map.keys()):
                self.info_map[neigh.id] = neigh

    def cg_config_rm_callback(self, msg):
        ######### SWARM TYPE CONSTRAINTS ####################
        constrains_type = msg.type_cnstr
        if constrains_type == "remove":
            id_value= msg.id
            self.remove_swarm_constraints(idpassed=id_value)

    def update_matrix_callback(self, msg):
        ''' Update the cost function matrix '''
        col = len(msg.data)/msg.row
        Psi_tmp = [msg.data[(i*col):((i+1)*col)] for i in range(msg.row) ]
        self.Psi = Psi_tmp 

    ############ TOKEN CALLBACK ######################### 
    def token_callback(self, msg):
        ''' Update vehicle's token '''
        #tmp = msg.data
        #self.token = tmp
        #rospy.loginfo('Token unlocked %s', self.token)

    ############ CG INFO CALLBACKS ######################
    def vehicle_info_callback(self, msg):
        ''' Update vehicle's current state '''
        if(not msg.id in self.info_map.keys()):
            self._lock.release() # Free reference governor after first state update
        self.info_map[msg.id] = msg
        
        
    def cg_reference_callback(self, msg):
        ''' Update vehicle's current reference (usually got from Planner) '''
        tmp = msg.data
        self.r = np.array((tmp[0],tmp[1],tmp[2]))
        self.r = np.reshape(self.r,(3,1))
        rospy.loginfo('New reference for CG %s', self.r)

    ############ CG MAIN COMPUTATION ####################
    def cg_controller_callback(self, enable):
        '''
        Compute a feasible reference every Tc_cg seconds:
        '''
        self._lock.acquire()
        while not self.server.is_preempt_requested():
            while(len(self.msg_queue) > 0):
                with self.mutex:
                    msg = self.msg_queue.pop(0)
                if(type(msg) == Constraint):
                    self.manage_cg_config_msg(msg)
            
            if self.token_enabled:
                xa = list(self.info_map[self.iden].x)
                gn = []
                for vehicle_id in self.info_map:
                    info = self.info_map[vehicle_id]
                    if(self.iden != vehicle_id):
                        xa.extend(list(info.x))
                        gn.extend(list(info.g))
                #rospy.loginfo(xa)
                tmp = np.array(xa)
                xa = np.reshape(tmp, (len(xa), 1))
                if(len(gn) > 0):
                    tmp = np.array(gn)
                    gn = np.reshape(tmp, (len(gn), 1))
                (ris, cost) = self.dcg.compute_cmd(xa, self.r, gn)
                if(len(ris) > 0):
                    rospy.loginfo('New solution %s', ris)
                    #rospy.loginfo('Nieb States %s', neig_state)
                    self.g = ris
                    self.g = np.reshape(self.g,(3,1)) 
                else:
                    rospy.loginfo('Older solution %s', self.g)  
                self.g_pub.publish((self.g.flatten()).tolist())
                # Release token
                #self.token_enabled = False                             
                self.token_release.publish(self.token_enabled)
                
            self.rate.sleep()
        # Cancel goal request received
        result = CGResult()
        result.status = 'computed' if np.linalg.norm(self.r-self.g) <= self.tracking_index else 'not computed'
        self.server.set_preempted(result,'cg unit stopped')




    def add_swarm_constraints(self, idpassed,proximity=-1,anticollision=-1):
        self.dcg.add_swarm_cnstr(idpassed,proximity,anticollision)

    def remove_swarm_constraints(self,idpassed):
        self.dcg.remove_swarm_cnstr(idpassed)

    def add_vehicle_constraints(self,position=[],speed=[], thrust=[]):
        self.dcg.add_vehicle_cnstr(position,speed,thrust)


    def loop(self):
        ''' Life cycle
        Compute the feasible command every Tc_cg seconds:
        u = argmin ((r-g)'Psi(r-g))
        '''
        while not rospy.is_shutdown():
            continue
            #rospy.spin()


if __name__ == '__main__':
    rg = ReferenceGovernor()
    rg.loop() 