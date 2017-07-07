'''
// ///////////////////////////////////// 
// Created on 06.09.2015

@author: DORF_RUST
// /////////////////////////////////////  
'''

from threading import *

import socket
import struct
import time

from messages.clientstates import *
import math


#===============================================================================
class ClientGeneric(): #former: inherited from Thread
    """ Client Socket: can be either a client for sending or for receiving data 
    the type has to be defined when creating the object with either receiver=True or sender=True
    the sender socket is only for sending data, the receiver socket only for receiving data, this cannot be mixed.
    """
    def __init__(self, parent, host, port,  **params):
        
        if 'sender' in params:
            if params['sender'] == True:
                print "SENDER"
                self.run = self._run_as_sender
                self.socket_type = "SND"
                
        if 'receiver' in params:
            if params['receiver'] == True:
                print "RECEIVER"
                self.run = self._run_as_receiver
                self.socket_type = "RCV"
        
        self.parent = parent
        self.stack_size = self.parent.stack_size
            
        self.host = host
        self.port = port
        
        self.running = False
        
        " For receiving messages "
        self.msg_rcv = ""
        self.msg_len_rcv = "" # test for checking to read in properly
                
        " Default byteorder is big-endian, needs to be checked for individual clients. "
        self.byteorder = "<" # "!" network, ">" big-endian, "<" for little-endian, see http://docs.python.org/2/library/struct.html
                
        " A practical way to store some data, from one operation call to another "
        self.storage = None

    #===========================================================================
    def run(self):
        """ dummy method: replaced by run_as_sender or run_as_receiver. """
        pass

    #===========================================================================
    def _run_as_receiver(self):
        #print "RECEIVER THREAD"
        while self.running:
            try:
                self.read()          
            except:
                #print "%s: ERROR in reading from socket! "   % (self.parent.identifier)                
                self.msg_rcv = ""
                self.msg_len_rcv = ""
                
            
            if self.parent.state != STATE_READY:
                self.parent.update()
                
    #===========================================================================
    def _run_as_sender(self):
        #print "SENDER THREAD"
        while self.running:
            try:
                if self.parent.get_snd_queue().empty() == False:
                    msg_type, msg = self.parent.get_from_snd_queue()
                    #print "TRY SENDING", (msg_type, msg)
                    self.send(msg_type, msg)
            except:
                print "%s: ERROR in Sender Socket! "   % (self.parent.identifier)       
                #self.parent.close() #self.close()

    #===========================================================================    
    def connect_to_server(self):
        self.running = False
        
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
            self.socket.settimeout(None) #self.socket.settimeout(1.0)
            self.socket.connect((self.host, self.port))
            
            self.thread = Thread(target=self.run, name=self.socket_type+self.parent.identifier)
            self.thread.daemon = False    
            self.running = True
            
            print "%s %s: Successfully connected to server %s on port %d." % (self.parent.identifier, self.socket_type, self.host, self.port)
            return True
        except:
            print "%s %s: Server %s is not available on port %d." % (self.parent.identifier, self.socket_type, self.host, self.port)
            self.running = False
            return False
    
    #===========================================================================
    def close(self):
        
        if self.running:
            self.socket.close()
            print "%s %s: Socket closed." % (self.parent.identifier, self.socket_type)
            self.running = False
            
            self.msg_rcv = ""
            self.msg_len_rcv = ""
            
            time.sleep(0.1)
            
            try:
                #print "IS ALIVE: ", self.thread.isAlive(), (self.parent.identifier, self.socket_type), activeCount(), enumerate()
                self.thread.join()
                #self.thread = Thread(target = self.run)
            except:
                print "%s %s: Cannot join thread." % (self.parent.identifier, self.socket_type)
        
    #===========================================================================
    def send(self, msg_id, msg = None):
        """ needs to be implemented by the child class """
        pass
    #===========================================================================
    def _send_command(self, cmd):
        """ needs to be implemented by the child class """
        pass
       
    #===========================================================================
    def get_header(self):
        '''
        The header of every message consists of: header sequence, timestamp_sec, timestamp_nanosec
        ''' 
        sequence = self.get_cmd_counter() # uint     
        frac, whole = math.modf(time.time())
        timestamp_sec, timestamp_nanosec = int(whole), int(frac*1000000000) # uint
        return [sequence, timestamp_sec, timestamp_nanosec]

    #===========================================================================
    def read(self):
        """ The transmission protocol for messages is:
        8 byte [message length] (= 1 * Unsigned 8 byte integer) # The message length is the length of the message without the message length and message type.
        8 byte [message type] (= 1 * Unsigned 8 byte integer)
        3 * 4 byte [header: sequence, timestamp_sec, timestamp_nanosec] (= 3 * Unsigned 4 byte integer)
        x byte [message] (according to type)
        """
        
        "1. read msg length" 
        if len(self.msg_len_rcv) < 8:      
            self.msg_len_rcv += self.socket.recv(8)
            if len(self.msg_len_rcv) < 8:
                return
            
        msg_length = struct.unpack_from(self.byteorder + "Q", self.msg_len_rcv, 0)[0]
        #print "MESSAGE LENGTH:", msg_length
        
        "2. read rest of msg according to msg_length"
        self.msg_rcv += self.socket.recv(msg_length + 8) # (message length + 8) = message body (msg + header) + message type
        if len(self.msg_rcv) < (msg_length + 8):
            return

        "3. unpack message type"
        msg_type = struct.unpack_from(self.byteorder + "Q", self.msg_rcv[:8], 0)[0]
        
        "4. unpack message header"
        self.parent.msg_counter_from_client = struct.unpack_from(self.byteorder + "I", self.msg_rcv[8:12], 0)[0]    
        self.parent.timestamp_sec_from_client = struct.unpack_from(self.byteorder + "I", self.msg_rcv[12:16], 0)[0]
        self.parent.timestamp_nanosec_from_client = struct.unpack_from(self.byteorder + "I", self.msg_rcv[16:20], 0)[0]
        
        "5. rest of the message will be passed on as raw message (if there is no message body, raw_message stays empty)"
        raw_msg = self.msg_rcv[20:]
        
        "6. reset msg_rcv + msg_len_rcv"
        self.msg_rcv = ""
        self.msg_len_rcv = ""
        
        "4. pass message id and raw message to process method "
        ok = self.process(msg_length, msg_type, raw_msg)
        return ok
    
    #===========================================================================
    def process(self, msg_len, msg_type, raw_msg):
        """ needs to be implemented by the child class """
        pass
    
    #=================================================================================
    def get_state(self):
        return self.parent.get_state() 
    #=================================================================================
    def publish_state(self, state):
        self.parent.publish_state(state) 
    #=================================================================================
    def set_cmd_counter(self, num):
        self.parent.set_cmd_counter(num)  
    #=================================================================================
    def get_cmd_counter(self):
        return self.parent.get_cmd_counter()