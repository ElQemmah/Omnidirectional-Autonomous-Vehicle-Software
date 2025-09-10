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
import serial
import time
import sys, os
import rospy
sys.path.insert(0,os.path.expanduser('~')+'/ares_ros_ws/src/asv/lib/')

from vectlib.vectornav_packet import Vn_packet

VN_LEN = 93



class Vn_interface():

    def __init__(self, verbose = False):
        """Constructor"""
        self.verbose = verbose

        if(self.verbose):
            print("Creating Vn serial reader...")

    
    """ Data buffer to store information """
    data_buffer = {
                  "time": list(),
                  }

    def connect_to_device(self, port ='/dev/ttyUSB0', baudrate = 115200):
        """ Search for an mti device and (if found) creat a connection with it
        Sequence of operation:
        1) Scan port searching for an mti device
        2) Open the right port and create a connection with
            required (outocomputed) baudrate """
        
        if(self.verbose):
            print("Opening port...")
        self.vn_board = serial.Serial(port, baudrate, timeout=5)
        if(self.verbose):
            print("Opening port2..")
        chunk = self.vn_board.read()
        if(self.verbose):
            print("Opening port3..")
        if(chunk == b''):
            raise RuntimeError("Connection with VectorNav broken")
        self.baudrate = baudrate
        self.port = port
        return True

    def configure_device_output(self, requested_output = ["free_acc"]):
        for item in requested_output:
            if(item == "free_acc"):
                 self.data_buffer["free_acc_X"] =  list()
                 self.data_buffer["free_acc_Y"] =  list()
                 self.data_buffer["free_acc_Z"] =  list()

    def read_next_packet(self):
        chunks = []
        chunk = self.vn_board.read()
        while(chunk != b'$'):
            chunk = self.vn_board.read()
        while chunk != b'\r':
            if chunk == b'':
                return -1
            chunks.append(chunk)
            chunk = self.vn_board.read()

        return Vn_packet(b''.join(chunks))

    def show_data(self, packet):
        s = ""
        if packet.packetType() == '$VNYMR':
            euler = packet.orientationEuler()
            s += " |Roll: %.2f" % euler[2] + ", Pitch: %.2f" % euler[1] + ", Yaw: %.2f " % euler[0]
            acc = packet.freeAcceleration()
            s += "Acc X: %.8f" % acc[0] + ",\t Acc Y: %.8f" % acc[1] + ",\t Acc Z: %.8f\n" % acc[2]
            #print(s)

        else:
            return 
        """ 
        OTHER PACKETS HANDLING TO BE IMPLEMENTED
        """
        print("%s\r" % s, end="", flush=True)

    def print_saved_data(self):
        for key in self.data_buffer:
            if(len(self.data_buffer[key]) > 0):
                print(key + ":")
                print(self.data_buffer[key])

    def clear_stored_data(self):
        for key in self.data_buffer:
            self.data_buffer[key].clear()
        if(self.verbose):
            print("Buffer cleared")

    def measure(self, duration_in_ms):
        output = {"time": list()}
        for key in self.data_buffer:
            output[key] = list()

        startTime = round(time.time() * 1000)
        while round(time.time() * 1000) - startTime <= duration_in_ms:
            # Retrieve a packet
            packet = self.read_next_packet()
            if(self.verbose):
                self.show_data(packet)
            if(packet.containsSampleTimeFine()):
                output["time"].append(packet.sampleTimeFine())
        return output

    def close_connection(self):
        if(self.verbose):
            print("Closing connection")
        self.vn_board.close()


