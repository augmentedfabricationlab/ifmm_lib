'''
. . . . . . . . . . . . . . . . . . . . . . 
.                                         .
.   <<      ><  <<      ><  <<  <<><><>   .
.   < ><   ><<  < ><   ><<  <<  <<        .
.   << >< > ><  << >< > ><  <<  <<><><>   .  
.   <<  ><  ><  <<  ><  ><  <<  <<        .
.   <<      ><  <<      ><  <<  <<        .
.   <<      ><  <<      ><  <<  <<        .
.                                         .
.             GKR 2016/17                 .
. . . . . . . . . . . . . . . . . . . . . .

Created on 19.11.2016

@author: kathrind
'''

from threading import *

import socket
import struct
import time
import math

from clientstates import client_states_str_array, READY, BUSY
from messagetypes import msg_types_str_array, MSG_CMD_BEND, MSG_CMD_INSERT_VERTICAL, MSG_CMD_RETRACT, \
                         MSG_CMD_RECEIVED, MSG_CMD_EXECUTED, MSG_DIGITAL_OUT, MSG_INFO, MSG_STRING, \
                         MSG_FLOAT_LIST, MSG_CMD_INSERT_AND_BEND, MSG_CMD_ROTATE_MOTOR_ABS, MSG_CMD_ROTATE_MOTOR_REL, \
                         MSG_CMD_INSERT_VERTICAL_BEF_TILT, MSG_CMD_BEND_AFTER_TILT


#import Rhino.Geometry as rg

class ArduinoClient(object):
    def __init__(self, host = "127.0.0.1", port=30001):
        self.host = host
        self.port = port
        self.byteorder = "<" # "!" network, ">" big-endian, "<" for little-endian, see http://docs.python.org/2/library/struct.html
        self.connected = False
        
        self.lock_counter = Lock()
        self.lock_state = Lock()
        
        self.msg_counter = 0
        self.msg_len_rcv = ""
        self.msg_rcv = ""
        
        self.info_msg = ""
        
        self.kill_wait_thread_flag = False
        
        self.set_state(READY)
        
        self.connection_success = False
        
        self.current_angle_val = 0
    #===========================================================================
    def connect(self):
        if self.connected == False:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(1)
            self.sock.connect((self.host, self.port))
            print "Connected to server %s on port %d" % (self.host, self.port)
            self.connected = True   
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
    def get_state(self, state):
        self.lock_state.acquire()
        state = self.state
        self.lock_state.release()
        return state
    
    #===========================================================================
    def __send(self, msg_type, msg = None, wait_for_response = True):
        """ send message according to message type """
        
        buf = None

        if msg_type == MSG_INFO:
            "[msg_type, msg_counter]"
            msg_snd_len = 4 + 4 #+4bytes = wait_for_respons
            params = [msg_snd_len, msg_type, self.get_msg_counter(), wait_for_response]
            buf = struct.pack(self.byteorder + "4i", *params)
        elif msg_type == MSG_DIGITAL_OUT:
            "[msg_type, msg_counter, int_do_num, int_do_state]"
            msg_snd_len = 4 + 8 + 4 #+4bytes = wait_for_respons
            params = [msg_snd_len, msg_type, self.get_msg_counter(), wait_for_response] + msg
            buf = struct.pack(self.byteorder + "4i" + "2i", *params)
        elif msg_type == MSG_CMD_INSERT_AND_BEND or msg_type == MSG_CMD_ROTATE_MOTOR_ABS or msg_type == MSG_CMD_ROTATE_MOTOR_REL:
            "[msg_type, msg_counter, int_val1, int_val2]"
            msg_snd_len = 4 + 8 + 4 #+4bytes = wait_for_respons
            params = [msg_snd_len, msg_type, self.get_msg_counter(), wait_for_response] + msg
            buf = struct.pack(self.byteorder + "4i" + "2i", *params)
        elif msg_type == MSG_STRING:
            "[msg_type, msg_counter, string]"
            msg_snd_len = len(msg) + 4 + 4 # + 4bytes = msg_counter +4bytes = wait_for_response            
            params = [msg_snd_len, msg_type, self.get_msg_counter(), wait_for_response] + [msg]
            buf = struct.pack(self.byteorder + "4i" + str(len(msg)) +  "s", *params)
        elif msg_type == MSG_CMD_BEND or msg_type == MSG_CMD_INSERT_VERTICAL or msg_type == MSG_CMD_RETRACT or msg_type == MSG_FLOAT_LIST or msg_type == MSG_CMD_INSERT_VERTICAL_BEF_TILT  or msg_type == MSG_CMD_BEND_AFTER_TILT:
            "[msg_type, msg_counter, float_values]"
            msg_snd_len = struct.calcsize(str(len(msg)) + "f") + 4 + 4 # + 4bytes = msg_counter   +4bytes = wait_for_respons
            params = [msg_snd_len, msg_type, self.get_msg_counter(), wait_for_response] + msg
            # print "params", params            
            buf = struct.pack(self.byteorder + "4i" + str(len(msg)) + "f", *params)     
        else:
            print "Message identifier unknown:  %s = %d, message: %s" % (msg_types_str_array[msg_type], msg_type, msg)
            return   
           
        if buf != None:
            self.sock.send(buf)
            print "Sent message: %s to server " % (msg_types_str_array[msg_type])

    #===========================================================================
    def __read(self):
        """ The transmission protocol for messages is:
        4 byte [message length] (= 1 * Unsigned 4 byte integer) # The message length is the length of the message without the message length and message type.
        4 byte [message type] (= 1 * Unsigned 4 byte integer)
        x byte [message] (according to type)
        """
        
        "1. read msg length"
        while len(self.msg_len_rcv) < 4:   
            self.msg_len_rcv += self.sock.recv(1)
         
        msg_length = struct.unpack_from(self.byteorder + "I", self.msg_len_rcv, 0)[0]
        #print "MESSAGE LENGTH:", msg_length
        
        "2. read rest of msg according to msg_length"
        while len(self.msg_rcv) < (msg_length + 4):    
            self.msg_rcv += self.sock.recv(1)

        "3. unpack message type"
        msg_type = struct.unpack_from(self.byteorder + "I", self.msg_rcv[:4], 0)[0]
        #print "MESSAGE TYPE:", msg_type
        
        "4. rest of the message will be passed on as raw message (if there is no message body, raw_message stays empty)"
        raw_msg = self.msg_rcv[4:]
        
        "5. reset msg_rcv + msg_len_rcv"
        self.msg_rcv = ""
        self.msg_len_rcv = ""
        
        "6. pass message id and raw message to process method "
        msg_type, msg = self.__process(msg_type, raw_msg)
        return msg_type, msg
    
    #===========================================================================
    def __process(self, msg_type, raw_msg):
        """ The transmission protocol for messages is 
        [length msg in bytes] [msg identifier] [other bytes which will be read out according to msg identifier] """
        
        if msg_type == MSG_CMD_RECEIVED:
            msg_counter = struct.unpack_from(self.byteorder + "i", raw_msg)[0]
            return (msg_type, msg_counter)
        elif msg_type == MSG_CMD_EXECUTED:
            msg_counter = struct.unpack_from(self.byteorder + "i", raw_msg)[0]
            return (msg_type, msg_counter)
        elif msg_type == MSG_INFO:
            msg_counter = struct.unpack_from(self.byteorder + "i", raw_msg[:4])[0]
            msg_string = raw_msg[4:]
            return (msg_type, (msg_counter,msg_string))
        else:
            print "%s: Message identifier unknown: %d, message: %s" % (self.parent.identifier, msg_type, raw_msg)
            return (False, False)

    def __get(self, msg_type, msg = None, wait_for_response = True):
        self.set_msg_counter()
        self.set_state(BUSY)
        self.__send(msg_type, msg, wait_for_response)
        
        success = False
        msg_received_by_arduino = False
        if wait_for_response:
            success = False
            while not success:
                
                # try to get a callback ba arduino, if message was received or not...
                if not msg_received_by_arduino:
                    #time.sleep(0.1)
                    try:
                        msg_type, msg = self.__read()
                        if msg_type == MSG_CMD_RECEIVED:
                            msg_received_by_arduino = True
                        else:
                            return (msg_type, msg, success)
                    except:
                        return (msg_type, msg, success)
                    
                # after receiving the acknowledge message from the arduino, try receiving the cmd executed msg    
                try:
                    msg_type, msg = self.__read()
                    success = True
                except:
                    print "waiting for response"
                    time.sleep(0.01)
                    
                    if self.kill_wait_thread_flag == True: # ------> thread can be killed from outside event
                        return (msg_type, msg, success)
                    
        else: # just try to receive the cmd_received_message
            success = False
            try:
                msg_type, msg = self.__read()
                if msg_type == MSG_CMD_RECEIVED:
                    success = True
                    return (msg_type, msg, success)
                else:
                    return (msg_type, msg, success)
            except:
                return (msg_type, msg, success)
        
        self.set_state(READY)
        return (msg_type, msg, success)
    
    def send_get_arduino_info(self):
        return self.__get(msg_type = MSG_INFO, msg = None, wait_for_response = True)
    
    
    # ===============================================================================================================================
    # motor angle calculations
    # ===============================================================================================================================
    def map_range(self, value,fromMin,fromMax,toMin,toMax):
        fromSpan = fromMax - fromMin
        toSpan = toMax - toMin
        valueScaled = float(value-fromMin)/float(fromSpan)
        return toMin + (valueScaled * toSpan)

    def get_motor_pos_value_from_angle(self, angle_degrees):
        '''returns the motor abs positions for an angle input
        please check: maybe needs to include a shift, depending on the zero position.
        '''
        mapped_value = self.map_range(angle_degrees, -90, 90, -200, 200)
        return mapped_value * 48./40. * -1.0
    
    def get_motor_steps_value_from_angle(self, angle_degrees):
        mapped_value = self.map_range(angle_degrees, -360, 360, -800, 800)
        return mapped_value
    
    # ===============================================================================================================================
    # communcation with Arduino
    # ===============================================================================================================================   
    def send_set_do(self, do = 0, state = 0, wait_for_response = True):
        """ command the arduino board to set a digital out to the given state. """
        return self.__get(msg_type = MSG_DIGITAL_OUT, msg = [do, state], wait_for_response = wait_for_response)
    
    def send_cmd_bend(self, angle, wait_for_response = True):
        """ send a bending command message to arduino with one float value. """
        motor_pos = self.get_motor_pos_value_from_angle(angle)
        return self.__get(msg_type = MSG_CMD_BEND, msg = [motor_pos], wait_for_response = wait_for_response)
    
    def send_cmd_rotate_motor_abs(self, angle, motor_type, wait_for_response = True):
        """ send a rotating command message to arduino with two int values. int1 = motortype, int2 = angle value """
        motor_pos = self.get_motor_pos_value_from_angle(angle)
        motor_type = 1 if motor_type == "front" else 2
        return self.__get(msg_type = MSG_CMD_ROTATE_MOTOR_ABS, msg = [int(motor_pos), int(motor_type)], wait_for_response = wait_for_response)
    
    def send_cmd_rotate_motor_rel(self, angle, motor_type, wait_for_response = True):
        """ send a rotating command message to arduino with two int values. int1 = motortype, int2 = angle value """
        motor_steps = self.get_motor_steps_value_from_angle(angle)
        motor_type = 1 if motor_type == "front" else 2
        return self.__get(msg_type = MSG_CMD_ROTATE_MOTOR_REL, msg = [int(motor_steps), int(motor_type)], wait_for_response = wait_for_response)
    
    def send_cmd_insert_vertical(self, f, wait_for_response = True):
        """ send a welding command message to arduino with one float value. 
        float value idetnifies a short or a long vertical wire to be inserted. """
        return self.__get(msg_type = MSG_CMD_INSERT_VERTICAL, msg = [f], wait_for_response = wait_for_response)
    
    def send_cmd_insert_vertical_bef_tilt(self, f, wait_for_response = True):
        """ send a welding command message to arduino with one float value. 
        float value idetnifies a short or a long vertical wire to be inserted. """
        return self.__get(msg_type = MSG_CMD_INSERT_VERTICAL_BEF_TILT, msg = [f], wait_for_response = wait_for_response)
    
    def send_cmd_bend_after_tilt(self, angle, wait_for_response = True):
        """ send a welding command message to arduino with one float value. 
        float value idetnifies a short or a long vertical wire to be inserted. """
        motor_pos = self.get_motor_pos_value_from_angle(angle)
        return self.__get(msg_type = MSG_CMD_BEND_AFTER_TILT, msg = [motor_pos], wait_for_response = wait_for_response)
 
    
    def send_cmd_retract(self, f, wait_for_response = True):
        """ send a bending command message to arduino with one float value. """
        return self.__get(msg_type = MSG_CMD_RETRACT, msg = [f], wait_for_response = wait_for_response)
    
    def send_cmd_insert_and_bend(self, vertical_type = 0, angle = 0, wait_for_response = True):
        """ command the arduino board to insert a vertical and bend afterwards """
        motor_pos = self.get_motor_pos_value_from_angle(angle)
        return self.__get(msg_type = MSG_CMD_INSERT_AND_BEND, msg = [vertical_type, motor_pos], wait_for_response = wait_for_response)

    def set_pneu_feeder(self, do_state):
        relay_pneu_feeder = 8
        msg_type, msg_counter, success = self.send_set_do(do = relay_pneu_feeder, state = do_state, wait_for_response = True)
        return success
        
    def set_pneu_welder(self, do_state):
        relay_pneu_welder = 9
        msg_type, msg_counter, success = self.send_set_do(do = relay_pneu_welder, state = do_state, wait_for_response = True)
        return success
        
    def set_pneu_cutter(self, do_state):
        relay_pneu_cutter = 12
        msg_type, msg_counter, success = self.send_set_do(do = relay_pneu_cutter, state = do_state, wait_for_response = True)
        return success
    
    def set_trigger_weld(self):
        relay_trigger_weld = 11
        msg_type, msg_counter, success = self.send_set_do(do = relay_trigger_weld, state = 1, wait_for_response = True)
        #time.sleep(0.1)
        time.sleep(0.1)
        msg_type, msg_counter, success = self.send_set_do(do = relay_trigger_weld, state = 0, wait_for_response = True)
        return success
    
    # =========================================================================================
    
    """
    # =================================================================================
    def wait_for_state_ready_old(self, delay = None):
        time.sleep(0.05)
        
        timeout = 0
        state = self.state
        while state != READY and timeout < 12000:
            state = self.state
            #print "state:", state
            time.sleep(0.05)
            timeout += 1

            if self.kill_wait_thread_flag == True:
                return False
        
        if state == READY:
            if delay:
                time.sleep(delay)
            return True
        else:
            return False"""
            
    # =================================================================================
    def wait_for_state_ready(self, delay = None):
        time.sleep(0.05)
        
        state = self.state
        while state != READY:
            state = self.state
            #print "state:", state
            time.sleep(0.05)

            if self.kill_wait_thread_flag == True:
                return False
        
        if state == READY:
            if delay:
                time.sleep(delay)
            return True
        else:
            return False
    
         

if __name__ == "__main__":
    aclient = ArduinoClient(host = "192.168.10.177", port=30001)
    time.sleep(0.5)
    
    aclient.connect()
    time.sleep(0.5)
    print "Connection: ", aclient.connected
    v1 = 0
    v2 = 50
    for i in range(1000):
        
        f = v1 if i%2==0 else v2
        #msg_type, msg_counter, success = aclient.send_cmd_bending(f=f)
        msg_type, msg, success = aclient.send_get_arduino_info()
        print "RECEIVED MSG FROM ARDUINO: "
        print "MSG TYPE: ", msg_types_str_array[msg_type]
        print "MSG: ", msg
        print "SUCCESS" if success==True else "NO SUCCESS"
        
        if not success:
            aclient.close()
            time.sleep(1)
            aclient.connect()
            
        #print "MSG COUNTER: ", msg_counter
    
        time.sleep(0.1)
    aclient.close()
    print "Connection: ", aclient.connected
    time.sleep(0.1)
    
    """
    for i in range(3):
        
        aclient.connect()
        time.sleep(0.5)
        print "Connection: ", aclient.connected
        
        msg_type, msg_counter = aclient.send_cmd(f=500)
        
        print "RECEIVED MSG FROM ARDUINO: "
        print "MSG TYPE: ", msg_types_str_array[msg_type]
        print "MSG COUNTER: ", msg_counter
        
        for i in range(3):
            time.sleep(0.5)
            msg_type, msg = aclient.send_get_arduino_info()
            print "RECEIVED MSG FROM ARDUINO: "
            print "MSG TYPE: ", msg_types_str_array[msg_type]
            print "MSG COUNTER: ", msg[0]
            print "MSG: ", msg[1]
            
            
            msg_type, msg_counter = aclient.send_set_do(do = 13, state = 0)
            print "RECEIVED MSG FROM ARDUINO: "
            print "MSG TYPE: ", msg_types_str_array[msg_type]
            print "MSG COUNTER: ", msg_counter
            
            msg_type, msg_counter = aclient.send_cmd(f=399.5)
            print "RECEIVED MSG FROM ARDUINO: "
            print "MSG TYPE: ", msg_types_str_array[msg_type]
            print "MSG COUNTER: ", msg_counter
            
        time.sleep(0.1)
        aclient.close()
        print "Connection: ", aclient.connected
        time.sleep(0.1)"""