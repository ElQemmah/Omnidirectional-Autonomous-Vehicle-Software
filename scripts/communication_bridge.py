#! /usr/bin/env python3
# -*- coding: utf-8 -*-

#  Copyright (C) 2021  ELTOS
#  This program is developed for Applicon srl by ELTOS.
#  Its copy, use, redistribution or modification is prohibited, or requires
#  you to ask for permission. All authorized modifications made to
#  the software are subject to the same conditions as the original software.
#  This program is provided as is: WITHOUT ANY WARRANTY; without even the
#  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
#  For a copy of the complete license please contact develop@applicon.it.

import rospy

# Import messages
from asv.msg import Plug_msg, Info_msg

# import socket programming library
import socket

# import thread module
from _thread import *
import threading

# import regular exspressions for message validation
import re

from std_msgs.msg import Bool, Int32

#HOST = "127.0.0.1"
HOST = "192.168.1.3"
RECEIVING_PORT = 12346
SENDING_PORT = 12347

MAP_ROS_UDP = {
    "INFO"                              : "INFO",
    "plugin-anticollision-proximity"    : "PIAP",
    "plugin-proximity-anticollision"    : "PIAP",
    "plugin-anticollision"              : "PIAN",
    "plugin-proximity"                  : "PIPX",
    "plugout-anticollision-proximity"   : "POAP",
    "plugout-proximity-anticollision"   : "POAP",
    "plugout-anticollision"             : "POAN",
    "plugout-proximity"                 : "POPX"
}

MAP_UDP_ROS = {
    "PIAP": ["plugin", ["anticollision", "proximity"]],   
    "PIAN": ["plugin", ["anticollision"]],              
    "PIPX": ["plugin", ["proximity"]],                  
    "POAP": ["plugout", ["anticollision", "proximity"]],     
    "POAN": ["plugout", ["anticollision"]],             
    "POPX": ["plugout", ["proximity"]]             
}

FORMAT = "$CCCC,I,XX.XX,XX.XX,XX.XX,XX.XX,XX.XX,XX.XX,XX.XX,XX.XX,XX.XX,XX.XX,XX.XX,XX.XX,X\n\r"
RE = "\$[A-Z]{4}" # Command $AAAA
RE += ",[0-9]" # Vehicle Index X
RE += "(,-{0,1}[0-9]{1,2}\.[0-9]{2}){9}" # State xa nine XX.XX elements
RE += "(,-{0,1}[0-9]{1,2}\.[0-9]{2}){3}" # Referece g three XX.XX
RE += "(,[0-9]\.{0,1}[0-9]{0,2}){0,2}" # Data
# RE += "\r\n" # END
RE = "(\$[A-Z]{4},[0-9])|(" + RE + ")"
BYTE_TO_READ = len(FORMAT.encode('ascii'))+10

class Communication_bridge:
    def __init__(self):
        ''' Init node '''
        # INIT NODE
        rospy.init_node('communication_bridge')
        self.rate = rospy.Rate(1/rospy.get_param('communication/freq', 0.1))

        # TOPICS
        rospy.Subscriber('output/plugin', Plug_msg, self.output_plugin_callback)
        rospy.Subscriber('output/info', Info_msg, self.output_info_callback)
        rospy.Subscriber('output/token', Bool, self.output_token_callback)
        self.input_plg = rospy.Publisher('input/plugin', Plug_msg, queue_size=10)
        self.input_infos = rospy.Publisher('input/info', Info_msg, queue_size=10)
        self.input_token = rospy.Publisher('input/token', Int32, queue_size=10)

        # PARAMETERS
        # State and reference to broadcast. Included a lock in order to wait for first state update
        self._lock = threading.Lock()
        self._lock.acquire()
        self.x = None
        self.g = None
        # Set node id and HOST
        num = rospy.get_param('~topic_prefix').split('asv') 
        self.id = int(num[-1])
        #self.HOST = "127.0.0." + str(self.id)
        self.HOST = HOST
        self.known_ids = {
            self.id :  HOST
        }
        # Connection for broadcasting packets
        self.out_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.out_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.out_socket.bind((self.HOST, SENDING_PORT))
        rospy.loginfo("Socket external message (vehicle {:d}) binded to address {:s} and port {:d}".format(self.id, self.HOST, SENDING_PORT))
        # Connection for receiving packets
        self.in_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.in_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.in_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.in_socket.bind(("<broadcast>", RECEIVING_PORT))
        rospy.loginfo("Socket internal message (vehicle {:d}) binded to address {:s} and port {:d}".format(self.id, self.HOST, RECEIVING_PORT))
        # Connection for receiving plugin packets
        self.plg_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.plg_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.plg_socket.bind((self.HOST, RECEIVING_PORT))
        rospy.loginfo("Socket plugin message (vehicle {:d}) binded to address {:s} and port {:d}".format(self.id, self.HOST, RECEIVING_PORT))

    def output_plugin_callback(self, msg):
        packet = self.plug_msg_to_string(msg)
        #rospy.loginfo("Sending packet plugin " + packet)
        self.out_socket.sendto(packet.encode('ascii'),  (self.known_ids[msg.id_dest], RECEIVING_PORT)) # immediately forward the plug request

    def output_token_callback(self, msg):
        packet = "$TOKN," + str(self.id)
        #rospy.loginfo("Sending packet token " + packet)
        for id_neig in self.known_ids:
            id = self.known_ids[id_neig]
            rospy.loginfo(id)
            self.out_socket.sendto(packet.encode('ascii'),  (id, RECEIVING_PORT)) # immediately forward the plug request
    
    def output_info_callback(self, msg):
        if(self.x == None): # Free brodcaster thread after the first info update
            self.x = msg.x
            self.id = msg.id
            self.g = msg.g
            self._lock.release()
        else:
            self.x = msg.x
            self.id = msg.id
            self.g = msg.g

    def string_to_ros_msg(self, msg):
        """Translate UDP strings into ROS messages"""
        data = msg.split(',')
        cmd = data[0][1:]
        if(cmd == "TOKN"):
            plg_pkt = int(data[1])
            return plg_pkt
        # Build info pkt
        info_pkt = Info_msg()
        info_pkt.id = int(data[1])
        x = list()
        for i in range(2, 11):
            x.append(float(data[i]))
        info_pkt.x = x
        g = list()
        for i in range(11, 14):
            g.append(float(data[i]))
        info_pkt.g = g
        if(cmd == 'INFO'):
            return info_pkt
        else: # if necessary build plugin packet
            plg_pkt = Plug_msg()
            if(not(cmd in MAP_UDP_ROS.keys())):
                rospy.logwarn("Attention!\n%s\n Received unknown command", msg)
                return None
            v = MAP_UDP_ROS[cmd]
            plg_pkt.rqst = v[0]
            plg_pkt.type = v[1]
            data_ex = list()
            for i in range(14, len(data)):
                data_ex.append(float(data[i]))
            plg_pkt.data = data_ex
            plg_pkt.info = info_pkt
            return plg_pkt

    def plug_msg_to_string(self, msg):
        '''Converting ros Plug_msg to viable udp string'''
        key = msg.rqst
        for tp in msg.type:
            key += "-" + tp
        # Start building udp message
        packet = "$" + MAP_ROS_UDP[key]
        packet += "," + str(msg.info.id)
        for i in range(len(msg.info.x)):
            packet += "," + "{:1.2f}".format(msg.info.x[i])
        for i in range(len(msg.info.g)):
            packet += "," + "{:1.2f}".format(msg.info.g[i])
        for i in range(len(msg.data)):
            packet += "," + str(msg.data[i])
        return packet

    def packet_manager(self, data, addr):
        '''Thread for managing incoming packets'''
        # data received from client
        msg = str(data.decode('ascii'))
        if(re.fullmatch(RE, msg, flags = re.ASCII) == None):
            rospy.logwarn("Attention!\n%s\n Message discarded because it is malformed", msg)
        else:
            ros_pkt = self.string_to_ros_msg(msg)
            if(type(ros_pkt) == Plug_msg):
                self.input_plg.publish(ros_pkt)
                self.known_ids[ros_pkt.info.id] = addr[0]
            elif(type(ros_pkt) == Info_msg):
                self.input_infos.publish(ros_pkt)
                self.known_ids[ros_pkt.id] = addr[0]
            elif(type(ros_pkt) == Int32 or type(ros_pkt) == int):
                #rospy.loginfo("DA PUBBLICARE TOKEN")
                self.input_token.publish(ros_pkt)
                
    def receiver(self, s):
        """Thread for managing incoming plug_in packets"""
        while not rospy.is_shutdown():
            # Listen for incoming packets
            data, addr = s.recvfrom(BYTE_TO_READ) # buffer size BYTE_TO_READ
            # Start a new thread for packets management
            if(addr[0] != self.HOST):
                #rospy.loginfo("Packet recived by {:s} : {:s}".format(str(addr[0]), str(addr[1])))
                #rospy.loginfo(data)
                start_new_thread(self.packet_manager, (data, addr))
        s.close()

    def loop(self):
        """Start listening for incoming UDP packets while broadcasting vehicle state"""
        start_new_thread(self.receiver, (self.in_socket,)) # Listen for broadcasted packets 
        start_new_thread(self.receiver, (self.plg_socket,)) # Listen for Plugin packets
        start_new_thread(self.receiver, (self.plg_socket,)) # Listen for Plugin packets
        # Broadcaster thread
        rospy.loginfo("Waiting for first state update...")
        self._lock.acquire() # Wait for the first info update
        rospy.loginfo(f"Broadcasting {self.id}")
        while not rospy.is_shutdown():
            packet = "$" + MAP_ROS_UDP["INFO"]
            packet += "," + str(self.id)           # id
            for i in range(len(self.x)):           # x
                packet += "," + "{:1.2f}".format(self.x[i])
            for i in range(len(self.g)):           # g
                packet += "," + "{:1.2f}".format(self.x[i])
            #rospy.loginfo("Sending packet " + packet)
            self.out_socket.sendto(packet.encode('ascii'),  ('<broadcast>', RECEIVING_PORT))
            self.rate.sleep()
        self.out_socket.close()


class Test:
    def __init__(self):
        pass
    def run(self):
        out_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        out_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        out_socket.bind(("127.0.0.23", SENDING_PORT))
        packet = "$PIPX,1,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.5"
        out_socket.sendto(packet.encode('ascii'),  ('127.0.0.1', RECEIVING_PORT)) # immediately forward the plug request


if __name__ == '__main__':
    bridge = Communication_bridge()
    bridge.loop()
