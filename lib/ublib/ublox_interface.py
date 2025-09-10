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
from pyubx2 import UBXMessage, UBXReader
from pyubx2.ubxtypes_configdb import  UBX_CONFIG_DATABASE
import warnings
import re
from serial import Serial
from io import BufferedReader
import threading
import time
from pynmeagps.nmeamessage import NMEAMessage

layers = {"RAM": 1, "BBR": 2, "Flash": 4}
CFG_PATTERN = "((RAM)|(BBR)|(Flash))( {0,5})(CFG-[^ \n\t]+)( {0,5})((0x([0-9]|[a-f]|[A-F]){1,10})|[0-9])"
SUPPORTED_BLOCK_TYPES = ["[set]", "[del]"]

PRE_SET = {
    "fixed_base"    : [],
    "moving_base"   : [],
    "rover"         : []
}

POS_MODE_DICT = {
                '': "Not received",
                "N": "No fix",
                "E": "estimated/dead reckoning fix", 
                "A": "autonomous GNSS fix", 
                "D": "differential GNSS fix", 
                "F": "RTK float", 
                "R": "RTK fixed"
            }

class Ubx_interface:
    def __init__(self):
        # Parameters
        self.msg_queue = {
            "RAM" : {"[del]": [], "[set]": []},
            "BBR" : {"[del]": [], "[set]": []},
            "Flash": {"[del]": [], "[set]": []}

        }
        self.ubx_board = None
        self.ubx_lock = threading.Lock()
        self.lat_lon = ('', '')
        self.pos_mode = ''
        self.status = ''
        self.time =  ''
        self.NS = ''
        self.EW = ''
        self.course_over_ground =  ''
        self.cog =  ''
        self.velN = ''
        self.velE = ''
        self.velD = ''
        self.alt0 = ''
        self.pos_modes = ''
        self.navigation_status = ''
        self.hAcc = ''
        self.sAcc = ''
        self.gpsHeadingM = ''
        self.gpsHeadingV = ''
        self.oAcc = ''
        self.ack = None

    def connect(self, port, baudrate = 460800):
        timeout = 1
        self.stream =  Serial(port, baudrate, timeout=timeout)
        # create UBXReader instance, reading only UBX messages
        self.ubx_board = UBXReader(BufferedReader(self.stream), protfilter=3)
        self.device_addr = port
        self.keep_reading = True
        self.reader = threading.Thread(target=self.read_from_board, args=())
        self.reader.start()

    def close(self):
        self.keep_reading = False
        self.reader.join()
        self.stream.close()

    def read_msg(self):
        """
        Reads, parses and prints out incoming UBX messages
        """
        # pylint: disable=unused-variable, broad-except
        with self.ubx_lock:
            try:
                (raw_data, parsed_data) = self.ubx_board.read()
            except Exception:
                (raw_data, parsed_data) = (None, None)

        return (raw_data, parsed_data)
    
    def wait_for_ack(self):
        if(self.ack == None):
            time.sleep(1)
        if(self.ack == None):
            return False
        ack = self.ack
        self.ack = None
        print(ack)
        return ack

    def send_msg(self, msg):
        """
        Send message to device
        """
        self.stream.write(msg.serialize())

    def parse_msg(self, msg, msg_type):
        admissible_cmd = re.search(CFG_PATTERN, msg)
        if(admissible_cmd):
            tokens = admissible_cmd.group(0).split()
            if(msg_type == "[set]"):
                transaction = 0 # transaction
                msg = UBXMessage.config_set(layers[tokens[0]], transaction, [(tokens[1].replace("-", "_"), int(tokens[2], base=16))])
            elif(msg_type == "[del]"):
                transaction = 0 # transaction
                msg = UBXMessage.config_del(layers[tokens[0]], transaction, (tokens[1].replace("-", "_")))
            elif(msg_type == "[get]"):
                position = 0 # position
                # unsigned integer representing number of items to be skipped before returning result
                # (used when number of matches for an individual query exceeds 64)
                msg = UBXMessage.config_poll(layers[tokens[0]], position, (tokens[1].replace("-", "_")))
            return msg
        else:
            raise ValueError('Message not matches pattern')

    def parse_conf_file(self, filename):
        # Read file
        with open(filename) as f:
            lines = f.readlines()
        # Process content
        for i in range(len(lines)):
            line = lines[i]
            # Skipp comments
            end = len(line)
            if("#" in line):
                end = line.index("#")
            line = line[:end]
            # Read instruction type block
            match_obj = re.search("\[.+\]", line)
            if(match_obj):
                block_type = match_obj.group(0)
                if(block_type in SUPPORTED_BLOCK_TYPES):
                    i += 1
                    while i < len(lines):
                        line = lines[i]
                        admissible_cmd = re.search(CFG_PATTERN, line)
                        if(admissible_cmd):
                            tokens = admissible_cmd.group(0).split()
                            if(not tokens[1].replace("-", "_") in UBX_CONFIG_DATABASE):
                                s = tokens[1].replace("-", "_")
                                warnings.warn(f"Attention {s} not recognized. Skipping.")
                                i += 1
                                continue
                            if(block_type == "[set]"):
                                self.msg_queue[tokens[0]][block_type].append((tokens[1].replace("-", "_"), int(tokens[2], base=16)))    
                            else:
                                self.msg_queue[tokens[0]][block_type].append(tokens[1].replace("-", "_"))
                        else:
                            if(re.search("\[.+\]", line)):
                                break
                        i += 1
                        
                else:
                    warnings.warn(f"Warning: block type {block_type} not handled. Skipping...")
        s = "Recognized commands:\n"
        for layer in self.msg_queue.keys():
            s += layer + ": "
            for msg_type in SUPPORTED_BLOCK_TYPES:
                recognized_msgs = len(self.msg_queue[layer][msg_type])
                s+= msg_type + ": " + str(recognized_msgs) + "; "
            s+= "\n"
        print(s)

    def send_all(self, transaction = 0):
        for layer in self.msg_queue.keys():
            for msg_type in SUPPORTED_BLOCK_TYPES:
                if(len(self.msg_queue[layer][msg_type]) > 0):
                    if(msg_type == "[set]"):
                        msg = UBXMessage.config_set(layers[layer], transaction, self.msg_queue[layer][msg_type])
                    else:
                        msg = UBXMessage.config_del(layers[layer], transaction, self.msg_queue[layer][msg_type])
                    self.send_msg(msg)
                    time.sleep(1)
                    if(not self.wait_for_ack()):
                        raise Exception(f"Message {msg} for layer {layer} was refused. Aborting.")
                    else:
                        self.msg_queue[layer][msg_type] = []
                        print("OK")
    
    def read_from_board(self):
        print("Start interfacing with ublox board")
        try:
            while True:
                if(self.keep_reading):
                    (raw_data, msg) = self.read_msg()
                    #print(msg)
                    if(type(msg) == NMEAMessage):
                        if(msg.msgID == "RMC"):
                            if(msg.status == "A"): # A: valid, V: invalid
                                self.lat_lon = (msg.lat, msg.lon)
                                self.time =  msg.time
                                self.NS = msg.NS
                                self.EW = msg.EW
                                #self.course_over_ground =  msg.spd
                                self.cog =  msg.cog, 
                                self.pos_mode = msg.posMode
                                self.navigation_status = msg.navStatus
                    elif(type(msg) == UBXMessage):
                        if(msg._ubxClass + msg._ubxID == b"\x05\x01"): # ACK
                            self.ack =  True
                        elif(msg._ubxClass + msg._ubxID == b"\x05\x00"): # NAK
                            self.ack = False
                        elif(msg._ubxClass + msg._ubxID == b"\x01\x07"): # UBX-NAV-PVT                        
                            #print(msg)
                            self.velN = msg.velN/1000
                            self.velE = msg.velE/1000
                            self.velD = msg.velD/1000
                            self.alt0 = msg.height/1000
                            self.hAcc = msg.hAcc/1000
                            self.sAcc = msg.sAcc/1000
                            self.gpsHeadingM = msg.headMot
                            self.gpsHeadingV = msg.headVeh
                            self.oAcc = msg.headAcc
                            #print(msg.velN)
                else:
                    print("stopping...")
                    break
        except Exception:
            print("stopping...")
            self.close()


    def config_fixed_base(self, lat, lon, alt):
        msg = self.parse_msg('Flash CFG-TMODE-MODE 2', '[set]') # write value 2 - FIXED to item id 0x20030001 in layer 2
        self.send_msg(msg)
        if not self.wait_for_ack(): return False
        msg = self.parse_msg('Flash CFG-TMODE-POS_TYPE 1', '[set]') # write value 1 - LLH to item id 0x20030002 in layer 2
        self.send_msg(msg)
        if not self.wait_for_ack(): return False
        msg = self.parse_msg(f'Flash CFG-TMODE-LAT {hex(int(lat*1e7))}', '[set]') # write value 393665100  0x1776da4c to item id 0x40030009 in layer 2
        self.send_msg(msg)
        if not self.wait_for_ack(): return False
        msg = self.parse_msg(f'Flash CFG-TMODE-LON {hex(int(lon*1e7))}', '[set]') # write value 162264010  0x9abf3ca to item id 0x4003000a in layer 2
        self.send_msg(msg)
        if not self.wait_for_ack(): return False
        msg = self.parse_msg(f'Flash CFG-TMODE-HEIGHT {hex(int(alt*1e2))}', '[set]') # write value 26500  0x6784 to item id 0x4003000a in layer 2
        self.send_msg(msg)
        if not self.wait_for_ack(): return False
        return True


    def __str__(self):
        num_lines = 9
        s = "                                            \n" * (num_lines)
        s += f"\x1B[{num_lines}A"
        s += f"Ublox_board connected to {self.device_addr}:\n" 
        s += f"latitude: {self.lat_lon[0]} {self.NS}\n"
        s += f"longitude: {self.lat_lon[1]} {self.EW}\n"
        s += f"cog: {self.course_over_ground}\n"
        s += f"vel north: {self.velN}\n"
        s += f"vel east: {self.velE}\n"
        s += f"vel down: {self.velD}\n"
        s += f"mode: {POS_MODE_DICT[self.pos_mode]}\n"
        s += f"\x1B[{num_lines}A"
        return s
