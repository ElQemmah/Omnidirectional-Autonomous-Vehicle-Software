#! /usr/bin/env python3
# -*- coding: utf-8 -*-

#  GNU License (C) 2021  ELTOS
#  This program is developed for Unical by Unical's researchers.
#  For a copy of the complete license please contact  {franco.torchiaro}{elqemmah.ay}@dimes.unical.it.

from pickle import STRING
import  rospy
import  numpy              as np
from    asv.msg            import Constraint, Neigh_data, Plug_msg, Info_msg
from std_msgs.msg import  Bool, Int32
import threading
AVAILABLE_CMD = ['ANTI', 'INFO', 'PROX']
R_1 = 10

class Communication_manager:
    def __init__(self):
        ''' Init node '''
        # INIT NODE
        rospy.init_node('communication_manager_node')

        # PARAMETERS
        topic_prefix = rospy.get_param('~topic_prefix')
        num = topic_prefix.split('asv')
        self.id = int(num[-1])
        self.rate = rospy.Rate(200)
        # Building local info for vehicle topic
        self.states = None  # made of x, xc = xa 
        self.ref = None # made of g
        self.g = None
        # Lock in order to wait for first state update
        self._lock = threading.Lock()
        self._lock.acquire()
         # Should be in form id-info associate 
        self.neib_info = {}  # in form  self.neib_info = {"id_neig": [xa,gn]}
        # TOPIC
        # Subscriber
        rospy.Subscriber('vehicle/info', Info_msg, self.vehicle_info_callback)
        rospy.Subscriber('input/plugin', Plug_msg, self.input_plug_in_callback)
        rospy.Subscriber('input/info', Info_msg, self.input_info_callback)
        rospy.Subscriber('input/token', Int32, self.input_token_callback)
        rospy.Subscriber('token/release', Bool, self.token_release_callback) # Token management
        # Publisher
        self.out_info = rospy.Publisher('output/info', Info_msg, queue_size=10)
        self.out_plugin = rospy.Publisher('output/plugin', Plug_msg, queue_size=10)
        self.token_out = rospy.Publisher('output/token', Bool, queue_size=10) # Token management
        self.cg_remove_cnstr = rospy.Publisher('cg/remove_cnstr', Constraint, queue_size=10) # CG configuration
        self.cg_add_cnstr = rospy.Publisher('cg/add_cnstr', Constraint, queue_size=10) # CG configuration
        self.neigh_info = rospy.Publisher('neigh/info', Neigh_data, queue_size=10)
        self.token_enable = rospy.Publisher('token/enable', Bool, queue_size=10)
        # bisogna in vehicle info avere le informazioni  relative alla g e poi nel dizionario aggiornare direttamente
        # l'intera sequenza con quella ricevuta (dentro la callback)

    def input_token_callback(self, msg):
        tmp = (self.neib_info[msg.data][0], True)
        self.neib_info[msg.data] = tmp 
    
    def token_release_callback(self, msg):
        self.token_out.publish(msg.data)
        for x in self.neib_info:
            tmp = (self.neib_info[msg.data][0], False)
            self.neib_info[x] = tmp  # SET TRUE IF COMPUTING DONE 

    ############## LOCAL INFO CALLBACKS #############################
    def vehicle_info_callback(self, msg):
        if(self.states == None):
            self.states = msg.x
            self.g = msg.g
            self._lock.release()
        else:
            self.states = msg.x
            self.g = msg.g

    def input_plug_in_callback(self, msg):
        cnstr_msg = Constraint()
        cnstr_msg.type_cnstr = msg.type
        cnstr_msg.data = msg.data
        cnstr_msg.info = msg.info
        if(msg.rqst == "plugin"):
            self.cg_add_cnstr.publish(cnstr_msg)
            l = list(msg.info.x)
            l.extend(list(msg.info.g))
            self.neib_info[msg.info.id] = (l, True)
        elif(msg.rqst == "plugout"):
            self.cg_remove_cnstr.publish(cnstr_msg)
            del self.neib_info[msg.id]
    
    def input_info_callback(self, msg): # Qui non so se fare il check dei raggi nella callback
        if(self.states == None):
            return
        #rospy.loginfo(f"Sono {self.id} ricevuto messaggi info da {msg.id}")
        if(self.is_plgin_necessary(msg.id, msg.x)):
            plg_msg = Plug_msg()
            plg_msg.rqst = "plugin"
            plg_msg.type = ["anticollision"]
            plg_msg.data = [1]
            plg_msg.id_dest = msg.id
            info_msg = Info_msg()
            info_msg.x = self.states
            info_msg.g = self.g
            info_msg.id = self.id
            plg_msg.info = info_msg
            self.out_plugin.publish(plg_msg)
            cnstr_msg = Constraint()
            cnstr_msg.type_cnstr = ["anticollision"]
            cnstr_msg.info = msg
            cnstr_msg.data = [1]
            self.cg_add_cnstr.publish(cnstr_msg)
            l = list(msg.x)
            l.extend(list(msg.g))
            self.neib_info[msg.id] = (l, False)
        if(msg.id in self.neib_info.keys()):
            l = list(msg.x)
            l.extend(list(msg.g))
            self.neib_info[msg.id] = (l, self.neib_info[msg.id][1])

    def token_in_callback(self, msg): # ?
        pass

    def is_plgin_necessary(self, id_n, x_neigh):
        x_neigh_np = np.array(x_neigh[0:2])
        local_x_np = np.array(self.states[0:2])
        if(np.linalg.norm(x_neigh_np - local_x_np) > R_1):
            return False
        if(np.linalg.norm(x_neigh_np - local_x_np) < R_1 and not(id_n in self.neib_info.keys())):
            return True
        return False

    def loop(self):
        ''' Life cycle
        Collect all data of vehicle
        '''
        self._lock.acquire()
        while not rospy.is_shutdown():
            ######### NEIG INFO TOPIC PUB ####################
            # DALLA MAPPA ESTRAI TUTTI GLI XA, LI CONCATENI E LI METTI IN XA 
            # DALLA MAPPA ESTRAI TUTTI I G, LI CONCATENI E LI METTI IN GN 
            data_packet = Neigh_data()
            enable_cg = True
            for x in self.neib_info:
                #rospy.loginfo(self.neib_info[x])
                info = Info_msg()
                value = self.neib_info[x][0]
                token = self.neib_info[x][1]        # SET TRUE IF COMPUTING DONE 
                info.x = value[0:9]
                info.g = value[9:12]
                info.id = x
                data_packet.info.append(info)
                if(not token):
                    enable_cg = False
            if(len(data_packet.info) > 0):
                self.neigh_info.publish(data_packet)

            self.token_enable.publish(enable_cg)

            info_msg = Info_msg()
            info_msg.id = self.id
            info_msg.x = self.states
            info_msg.g = self.g
            self.out_info.publish(info_msg)
            self.rate.sleep()
        
if __name__ == '__main__':  
    r = Communication_manager()
    r.loop() 