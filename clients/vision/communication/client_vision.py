'''
. . . . . . . . . . . . . . . . . . . . . . 
.                                         .
.   <<  <<><><>  <<      ><  <<      ><   .
.   <<  <<       < ><   ><<  < ><   ><<   .
.   <<  <<><><>  << >< > ><  << >< > ><   .  
.   <<  <<       <<  ><  ><  <<  ><  ><   .
.   <<  <<       <<      ><  <<      ><   .
.   <<  <<       <<      ><  <<      ><   .
.                                         .
.             GKR 2016/17                 .
. . . . . . . . . . . . . . . . . . . . . .

Created on 21.02.2017

@author: kathrind
'''

from threading import *

import socket
import struct
import time
import math

from ifmm_lib.clients.vision.communication.messagetypes import MSG_VISION_GET_CURRENT_WIRE_LINE, MSG_VISION_CURRENT_WIRE_LINE, MSG_VISION_GET_CURRENT_WIRE_LINE_1LINEINPUT, MSG_VISION_CURRENT_WIRE_LINE_1LINEOUTPUT, MSG_VISION_GET_CURRENT_WIRE_LINE_3LINEINPUT, MSG_VISION_CURRENT_WIRE_LINE_3LINEOUTPUT
from ifmm_lib.clients.vision.communication.messagetypes import MSG_VISION_SEND_CURRENT_ARM_POSE, MSG_VISION_SEND_SUCCESS, MSG_VISION_SUCCESS, MSG_VISION_GET_CURRENT_BASE_POSE, MSG_VISION_CURRENT_BASE_POSE, MSG_VISION_GET_CURRENT_BASE_POSE_2, MSG_VISION_CURRENT_BASE_POSE_2
from ifmm_lib.clients.vision.communication.messagetypes import vision_msg_type_dict
from ifmm_lib.clients.clientstates import READY, BUSY

#import Rhino.Geometry as rg

class ClientVision(object):
    def __init__(self, host='192.168.125.4', port=20003):
        self.host = host
        self.port = port
        
        self.byteorder = "<" # "!" network, ">" big-endian, "<" for little-endian, see http://docs.python.org/2/library/struct.html
        self.connected = False
        
        self.msg_counter_from_client = 0
        self.timestamp_sec_from_client = 0
        self.timestamp_nanosec_from_client = 0
        
        self.lock_counter = Lock()
        self.lock_state = Lock()
        
        self.msg_counter = 0
        self.msg_len_rcv = ""
        self.msg_rcv = ""
        
        self.info_msg = ""
        
        #self.kill_wait_thread_flag = False
        self.wait_flag = True
        
        self.state = READY
        
        self.connection_success = False
    
    #===========================================================================
    def is_connected(self):
        return self.connection_success & self.connected
        
    #===========================================================================
    def connect(self):
        if self.connected == False:
            # connect 
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(0.1)
            self.sock.connect((self.host, self.port))
            print "Connected to server %s on port %d" % (self.host, self.port)
            self.connected = True

    #===========================================================================
    def reset(self):
        if self.connected:
            self.close()
        self.connect()
        
        #msg_type, msg, success = self.get_current_wire_line()
        msg_type, msg, success = self.send_current_arm_pose([0.11626742449272023, -1.1021150988729778, 0.11692848402380923, -0.12930746011595484, 0.38686473260011522, 0.90563749640245828, 0.11591369435269336])

        if success:
            self.connection_success = True
            self.connected = True
            self.set_state(READY)
        else:
            self.connection_success = False
            self.connected = False
             
    #===========================================================================    
    def close(self):
        self.sock.close()
        self.connected = False
    #===========================================================================
    def set_msg_counter(self): 
        self.lock_counter.acquire()
        self.msg_counter += 1
        self.lock_counter.release()
    #===========================================================================
    def get_msg_counter(self):
        self.lock_counter.acquire()
        c = self.msg_counter
        self.lock_counter.release()
        return c
    # =================================================================================
    def set_state(self, state):
        self.lock_state.acquire()
        self.state = state
        self.lock_state.release()
    # =================================================================================
    def get_state(self):
        self.lock_state.acquire()
        state = self.state
        self.lock_state.release()
        return state
    
    # =================================================================================
    def set_wait_flag(self, state):
        self.wait_flag = state
    
    #===========================================================================
    def get_header(self):
        '''
        The header of every message consists of: header sequence, timestamp_sec, timestamp_nanosec
        ''' 
        sequence = self.get_msg_counter() # uint     
        frac, whole = math.modf(time.time())
        timestamp_sec, timestamp_nanosec = int(whole), int(frac*1000000000) # uint
        #print [sequence, timestamp_sec, timestamp_nanosec]
        return [sequence, timestamp_sec, timestamp_nanosec]
    
    #===========================================================================
    def _send_get_current_wire_line(self, msg_type):
        """ for commanding base
        8 byte [message length] (Q = 1 * Unsigned 8 byte integer) # The message length is the length of the message without the message length and message type.
        8 byte [message type] (Q = 1 * Unsigned 8 byte integer)
        3 * 4 byte [header: sequence, timestamp_sec, timestamp_nanosec] (I = 3 * Unsigned 4 byte integer)
        x byte [message] (according to cmd)
        """
        
        msg_snd_len = 12 #+ struct.calcsize(str(len(msg)) + "f") # = 12 because msg len is zero
        
        params = [msg_snd_len, msg_type] + self.get_header()
        buf = struct.pack(self.byteorder + "2Q" +  "3I", *params)
        self.sock.send(buf)
        
        print "Sent message: %s to server " % (vision_msg_type_dict[msg_type])
        
    #===========================================================================
    def _send_get_current_wire_line_1lineinput(self, msg_type, msg):
        """ for commanding base
        8 byte [message length] (Q = 1 * Unsigned 8 byte integer) # The message length is the length of the message without the message length and message type.
        8 byte [message type] (Q = 1 * Unsigned 8 byte integer)
        3 * 4 byte [header: sequence, timestamp_sec, timestamp_nanosec] (I = 3 * Unsigned 4 byte integer)
        x byte [message] (according to cmd)
        in this case: msg = [d1, d2, d3, d4, d5, d6] = [line.From.X, line.From.Y, line.From.Z, line.To.X, line.To.Y, line.To.Z]
        """
        
        msg_snd_len = 12 + struct.calcsize(str(len(msg)) + "d") # = 12 if msg len is zero + len msg
        
        params = [msg_snd_len, msg_type] + self.get_header() + msg
        buf = struct.pack(self.byteorder + "2Q" +  "3I" + str(len(msg)) + "d", *params)
        self.sock.send(buf)
        
        print "Sent message: %s to server " % (vision_msg_type_dict[msg_type])
        
    #===========================================================================
    def _send_get_current_wire_line_3lineinput(self, msg_type, msg):
        """ for commanding base
        8 byte [message length] (Q = 1 * Unsigned 8 byte integer) # The message length is the length of the message without the message length and message type.
        8 byte [message type] (Q = 1 * Unsigned 8 byte integer)
        3 * 4 byte [header: sequence, timestamp_sec, timestamp_nanosec] (I = 3 * Unsigned 4 byte integer)
        x byte [message] (according to cmd)
        in this case: msg = [d1 - d18] = [line.From.X, line.From.Y, line.From.Z, line.To.X, line.To.Y, line.To.Z * 3]
        """
        #print len(msg)
        msg_snd_len = 12 + 4 + struct.calcsize(str(len(msg)-1) + "d") # = 12 if msg len is zero // + 4bytes for the unsigned int + len msg - 1 
        
        params = [msg_snd_len, msg_type] + self.get_header() + msg
        buf = struct.pack(self.byteorder + "2Q" +  "4I" + str(len(msg)-1) + "d", *params)
        self.sock.send(buf)
        
        print "Sent message: %s to server " % (vision_msg_type_dict[msg_type])
        
    #===========================================================================
    def _send_current_arm_pose(self, msg_type, msg):
        """ for commanding base
        8 byte [message length] (Q = 1 * Unsigned 8 byte integer) # The message length is the length of the message without the message length and message type.
        8 byte [message type] (Q = 1 * Unsigned 8 byte integer)
        3 * 4 byte [header: sequence, timestamp_sec, timestamp_nanosec] (I = 3 * Unsigned 4 byte integer)
        x byte [message] (according to cmd)
        in this case: msg = [x,y,z,qw,qx,qy,qz] = pose quaternion
        """
        
        msg_snd_len = 12 + struct.calcsize(str(len(msg)) + "d") # = 12 if msg len is zero + len msg
        
        params = [msg_snd_len, msg_type] + self.get_header() + msg
        buf = struct.pack(self.byteorder + "2Q" +  "3I" + str(len(msg)) + "d", *params)
        self.sock.send(buf)
        
        print "Sent message: %s to server " % (vision_msg_type_dict[msg_type])
        
    #===========================================================================
    def _send_get_current_base_pose(self, msg_type):
        """ for commanding base
        8 byte [message length] (Q = 1 * Unsigned 8 byte integer) # The message length is the length of the message without the message length and message type.
        8 byte [message type] (Q = 1 * Unsigned 8 byte integer)
        3 * 4 byte [header: sequence, timestamp_sec, timestamp_nanosec] (I = 3 * Unsigned 4 byte integer)
        x byte [message] (according to cmd)
        """
        
        msg_snd_len = 12 #+ struct.calcsize(str(len(msg)) + "f") # = 12 because msg len is zero
        
        params = [msg_snd_len, msg_type] + self.get_header()
        buf = struct.pack(self.byteorder + "2Q" +  "3I", *params)
        self.sock.send(buf)
        
        print "Sent message: %s to server " % (vision_msg_type_dict[msg_type])
    
        #===========================================================================
    def _send_get_current_base_pose_2(self, msg_type, msg):
        """ for commanding base
        8 byte [message length] (Q = 1 * Unsigned 8 byte integer) # The message length is the length of the message without the message length and message type.
        8 byte [message type] (Q = 1 * Unsigned 8 byte integer)
        3 * 4 byte [header: sequence, timestamp_sec, timestamp_nanosec] (I = 3 * Unsigned 4 byte integer)
        x byte [message] (according to cmd)
        in this case: msg = [x,y,z,qw,qx,qy,qz] = pose quaternion
        """
        
        msg_snd_len = 12 + struct.calcsize(str(len(msg)) + "d") # = 12 if msg len is zero + len msg
        
        params = [msg_snd_len, msg_type] + self.get_header() + msg
        buf = struct.pack(self.byteorder + "2Q" +  "3I" + str(len(msg)) + "d", *params)
        self.sock.send(buf)
        
        print "Sent message: %s to server " % (vision_msg_type_dict[msg_type])
        
    #===========================================================================
    def __send(self, msg_type, msg = None):
        """ send message according to message type """
        
        buf = None

        if msg_type == MSG_VISION_GET_CURRENT_WIRE_LINE:
            self._send_get_current_wire_line(msg_type)
            
        elif msg_type == MSG_VISION_GET_CURRENT_WIRE_LINE_1LINEINPUT:
            self._send_get_current_wire_line_1lineinput(msg_type, msg)
        
        elif msg_type == MSG_VISION_GET_CURRENT_WIRE_LINE_3LINEINPUT:
            self._send_get_current_wire_line_3lineinput(msg_type, msg)
        
        elif msg_type == MSG_VISION_SEND_CURRENT_ARM_POSE:
            self._send_current_arm_pose(msg_type, msg)
        
        elif msg_type == MSG_VISION_GET_CURRENT_BASE_POSE:
            self._send_get_current_base_pose(msg_type)
        
        elif msg_type == MSG_VISION_GET_CURRENT_BASE_POSE_2:
            self._send_get_current_base_pose_2(msg_type, msg)
            
        else:
            print "Message identifier unknown:  %s = %d, message: %s" % (vision_msg_type_dict[msg_type], msg_type, msg)
            return   
           
        if buf != None:
            self.sock.send(buf)
            print "Sent message: %s to server " % (vision_msg_type_dict[msg_type])
            
    #===========================================================================

    #===========================================================================
    def __read(self):  
        """ The transmission protocol for messages is:
        8 byte [message length] (= 1 * Unsigned 8 byte integer) # The message length is the length of the message without the message length and message type.
        8 byte [message type] (= 1 * Unsigned 8 byte integer)
        3 * 4 byte [header: sequence, timestamp_sec, timestamp_nanosec] (= 3 * Unsigned 4 byte integer)
        x byte [message] (according to type)
        """
        
        "1. read msg length"
        
        max_count = 100
        counter = 0
        while len(self.msg_len_rcv) < 8:
            print "try reading msg_len"  
            self.msg_len_rcv += self.sock.recv(8)
            
            if self.wait_flag == False: # ------> thread can be killed from outside event
                return (None, None, False)
            
            counter += 1
            time.sleep(0.01)
            if counter > max_count:
                return (None, None, False)
            
            
         
        msg_length = struct.unpack_from(self.byteorder + "Q", self.msg_len_rcv, 0)[0]
        #print "MESSAGE LENGTH:", msg_length
        
        "2. read rest of msg according to msg_length"
        
        max_count = 100
        counter = 0
        while len(self.msg_rcv) < (msg_length + 8):  # (message length + 8) = message body (msg + header) + message type  
            self.msg_rcv += self.sock.recv(1)
            
            if self.wait_flag == False: # ------> thread can be killed from outside event
                return (None, None, False)
            
            counter += 1
            time.sleep(0.01)
            if counter > max_count:
                return (None, None, False)

        "3. unpack message type"
        msg_type = struct.unpack_from(self.byteorder + "Q", self.msg_rcv[:8], 0)[0]
        #print "MESSAGE TYPE:", msg_type
        
        "4. unpack message header"
        self.msg_counter_from_client = struct.unpack_from(self.byteorder + "I", self.msg_rcv[8:12], 0)[0]    
        self.timestamp_sec_from_client = struct.unpack_from(self.byteorder + "I", self.msg_rcv[12:16], 0)[0]
        self.timestamp_nanosec_from_client = struct.unpack_from(self.byteorder + "I", self.msg_rcv[16:20], 0)[0]
        
        "5. rest of the message will be passed on as raw message (if there is no message body, raw_message stays empty)"
        raw_msg = self.msg_rcv[20:]
        
        "6. reset msg_rcv + msg_len_rcv"
        self.msg_rcv = ""
        self.msg_len_rcv = ""
        
        "7. pass message id and raw message to process method "
        msg_type, msg = self.__process(msg_type, raw_msg)
        return (msg_type, msg, True)
    
    #===========================================================================
    def __process(self, msg_type, raw_msg):
        """ The transmission protocol for messages is 
        [length msg in bytes] [msg identifier] [other bytes which will be read out according to msg identifier] """
        
        if msg_type == MSG_VISION_CURRENT_WIRE_LINE:
            msg = struct.unpack_from(self.byteorder + "6d", raw_msg) #[0]
            return (msg_type, msg)
        
        elif msg_type ==  MSG_VISION_CURRENT_WIRE_LINE_1LINEOUTPUT:
            msg = struct.unpack_from(self.byteorder + "6d", raw_msg) #[0]
            return (msg_type, msg)
        
        elif msg_type ==  MSG_VISION_CURRENT_WIRE_LINE_3LINEOUTPUT:
            msg = struct.unpack_from(self.byteorder + "18d", raw_msg) #[0]
            return (msg_type, msg)
        
        elif msg_type ==  MSG_VISION_SUCCESS:
            msg = []
            return (msg_type, msg)
        
        elif msg_type == MSG_VISION_CURRENT_BASE_POSE:
            msg = struct.unpack_from(self.byteorder + "7d", raw_msg) #[0]
            return (msg_type, msg)
        
        elif msg_type == MSG_VISION_CURRENT_BASE_POSE_2:
            msg = struct.unpack_from(self.byteorder + "7d", raw_msg) #[0]
            return (msg_type, msg)
        
        else:
            print "Message identifier unknown: %d, message: %s" % (msg_type, raw_msg)
            return (False, False)
    
    #===========================================================================
    def __get(self, msg_type, msg = None):
        self.set_msg_counter()
        self.set_state(BUSY)
        self.__send(msg_type, msg)
        

        success = False
        
        max_count = 500 # max wating time = 5 sec
        counter = 0
        while not success:
                
            # try receiving the cmd executed msg    
            try:
                msg_type, msg, success = self.__read()
                #success = True
            except:
                print "waiting for response"
                time.sleep(0.01)
                
                """
                if self.kill_wait_thread_flag == True: # ------> thread can be killed from outside event
                    return (msg_type, msg, success)"""
                
                if self.wait_flag == False: # ------> thread can be killed from outside event
                    return (msg_type, msg, success)
            
            counter += 1
            
            if counter > max_count:
                return (msg_type, msg, success)
                
        
        self.set_state(READY)
        return (msg_type, msg, success)
    
    #===========================================================================
    # MESSAGES TO TVISION   
    #===========================================================================
    #===========================================================================
    def get_current_wire_line(self):
        return self.__get(msg_type = MSG_VISION_GET_CURRENT_WIRE_LINE, msg = None)
    
    #===========================================================================
    def get_current_line_estimation_1lineinput(self, msg):
        return self.__get(msg_type = MSG_VISION_GET_CURRENT_WIRE_LINE_1LINEINPUT, msg = msg)
    
    #===========================================================================
    def get_current_line_estimation_3lineinput(self, msg, estimation_type = 2):
        ''' the estimation type tells which wire to match:
        1 = front wire, 2 is back wire, (3  would be both)'''
        
        # TODO
        # should be:
        # msg = [estimation_type] + msg
        
        msg.insert(0, estimation_type) # add the integer to the msg list as the first entry
        
        msg_type, msg, success = self.__get(msg_type = MSG_VISION_GET_CURRENT_WIRE_LINE_3LINEINPUT, msg = msg)
        if success:
            print msg
            print len(msg)
            msg_as_list = list(msg)
            i = 0
            for item in msg_as_list:
                if(i % 6 == 0 or i % 6 == 1 or i % 6 == 2):
                    msg_as_list[i] = item * 1000 
                i+=1
                
                
            msg = msg_as_list
            #for item in enumerate(msg_as_list):
            #    print item    
            #msg = [item * 1000 if (index % 6 == 0 or index % 6 == 1 or index % 6 == 2) else item for index, item in enumerate(msg)]
            
        return (msg_type, msg, success)
    
    #===========================================================================
    def send_current_arm_pose(self, msg):
        return self.__get(msg_type = MSG_VISION_SEND_CURRENT_ARM_POSE, msg = msg)
    
    #===========================================================================
    def get_current_base_pose(self):
        msg_type, msg, success = self.__get(msg_type = MSG_VISION_GET_CURRENT_BASE_POSE, msg = None)
        if success:
            msg = [v for v in msg]
            msg[:3] = [v*1000. for v in msg[:3]]
        
        return (msg_type, msg, success)
    
    #===========================================================================
    def get_current_base_pose_2(self, msg=[0,0,0,0,0,0,0]):
        # the msg is the current pose of the robot
        msg_type, msg, success = self.__get(msg_type = MSG_VISION_GET_CURRENT_BASE_POSE_2, msg = msg)
        if success:
            msg = [v for v in msg]
            msg[:3] = [v*1000. for v in msg[:3]]
        
        return (msg_type, msg, success)
    
    