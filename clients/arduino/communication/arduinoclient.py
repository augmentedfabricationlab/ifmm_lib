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

from ifmm_lib.clients.clientstates import states_dict, READY, BUSY
from ifmm_lib.clients.arduino.communication.messagetypes import arduino_msg_type_dict, MSG_CMD_RECEIVED, MSG_CMD_EXECUTED, MSG_DIGITAL_OUT, MSG_INFO, MSG_STRING, \
                         MSG_FLOAT_LIST, MSG_CMD_ROTATE_MOTOR_ABS, MSG_CMD_ROTATE_MOTOR_REL, MSG_CMD_FEED_WIRE, MSG_CMD_ELECTRODES, \
                         MSG_CMD_NIPPERS, MSG_CMD_WELD, MSG_CMD_ROUTINE_FEED_CUT_WELD, MSG_CMD_BEND, MSG_CMD_RESET_ENCODER, MSG_CMD_GET_ENCODER_VAL
from ifmm_lib.useful import map_range

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
        
        self.wait_flag = True
        
        self.set_state(READY)
        
        self.connection_success = False
        
        #self.angle_per_step_m_front = (1.8/4)/8 # --> if motor is configured for micro steps
        #self.angle_per_step_m_back = (1.8/4)/9 * 29./34. # --> if motor is configured for micro steps
        self.angle_per_step_m_front = (1.8)/15
        self.angle_per_step_m_back = (1.8)/9 * 29./34.
        self.angle_per_step_m_feeder = (1.8/4)
        
    
    #===========================================================================
    def is_connected(self):
        return self.connection_success & self.connected
    
    #===========================================================================    
    def connect(self):
        if self.connected == False:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(1) #self.socket.settimeout(1.0)
                self.sock.connect((self.host, self.port))
                print "Connected to server %s on port %d" % (self.host, self.port)
                self.connected = True
                return True
            except:
                print "Connection to server %s on port %d not available" % (self.host, self.port)
                self.connected = False
                return False
                
    #===========================================================================
    def reset(self):
        if self.connected:
            self.close()
        ok = self.connect()
        
        if ok:
            msg_type, msg, success = self.send_get_arduino_info()
            if success:
                self.connection_success = True
                self.connected = True
                self.set_state(READY)
            else:
                self.connection_success = False
                self.connected = False
    
    #===========================================================================
    def reset_old(self):
        if self.connected:
            self.close()
        self.connect()
        
        msg_type, msg, success = self.send_get_arduino_info()
        if success:
            self.connection_success = True
            self.connected = True
            self.set_state(READY)
        else:
            self.connection_success = False
            self.connected = False
        
    #===========================================================================
    def connect_old(self):
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
    def get_state(self):
        self.lock_state.acquire()
        state = self.state
        self.lock_state.release()
        return state
    # =================================================================================
    def set_wait_flag(self, state):
        self.wait_flag = state
    
    #===========================================================================
    def __send(self, msg_type, msg = None, wait_for_response = True):
        """ send message according to message type """
        
        buf = None
        #print msg_type
        #print MSG_CMD_ELECTRODES
        
        if msg_type == MSG_INFO:
            "[msg_type, msg_counter]"
            msg_snd_len = 4 + 4 #+4bytes = wait_for_respons
            params = [msg_snd_len, msg_type, self.get_msg_counter(), wait_for_response]
            buf = struct.pack(self.byteorder + "4i", *params)
        elif msg_type == MSG_CMD_WELD or msg_type == MSG_CMD_RESET_ENCODER or msg_type == MSG_CMD_GET_ENCODER_VAL: #empty message
            "[msg_type, msg_counter]"
            msg_snd_len = 4 + 4 #+4bytes = wait_for_respons
            params = [msg_snd_len, msg_type, self.get_msg_counter(), wait_for_response]
            buf = struct.pack(self.byteorder + "4i", *params)          
        elif msg_type == MSG_DIGITAL_OUT:
            "[msg_type, msg_counter, int_do_num, int_do_state]"
            msg_snd_len = 4 + 8 + 4 #+4bytes = wait_for_respons
            params = [msg_snd_len, msg_type, self.get_msg_counter(), wait_for_response] + msg
            buf = struct.pack(self.byteorder + "4i" + "2i", *params)
        elif msg_type == MSG_CMD_ROTATE_MOTOR_ABS or msg_type == MSG_CMD_ROTATE_MOTOR_REL or msg_type == MSG_CMD_BEND:
            "[msg_type, msg_counter, int_val1, int_val2]"
            msg_snd_len = 4 + 8 + 4 #+4bytes = wait_for_respons
            params = [msg_snd_len, msg_type, self.get_msg_counter(), wait_for_response] + msg
            buf = struct.pack(self.byteorder + "4i" + "2i", *params)
        elif msg_type == MSG_CMD_FEED_WIRE or msg_type == MSG_CMD_ELECTRODES or msg_type == MSG_CMD_NIPPERS or msg_type == MSG_CMD_ROUTINE_FEED_CUT_WELD:
            #print "HERE"
            "[msg_type, msg_counter, int_val1]"
            msg_snd_len = 4 + 4 + 4 #+4bytes = wait_for_respons
            params = [msg_snd_len, msg_type, self.get_msg_counter(), wait_for_response] + msg
            buf = struct.pack(self.byteorder + "4i" + "i", *params)
        elif msg_type == MSG_STRING:
            "[msg_type, msg_counter, string]"
            msg_snd_len = len(msg) + 4 + 4 # + 4bytes = msg_counter +4bytes = wait_for_response            
            params = [msg_snd_len, msg_type, self.get_msg_counter(), wait_for_response] + [msg]
            buf = struct.pack(self.byteorder + "4i" + str(len(msg)) +  "s", *params)
        elif msg_type == MSG_FLOAT_LIST:
            "[msg_type, msg_counter, float_values]"
            msg_snd_len = struct.calcsize(str(len(msg)) + "f") + 4 + 4 # + 4bytes = msg_counter   +4bytes = wait_for_respons
            params = [msg_snd_len, msg_type, self.get_msg_counter(), wait_for_response] + msg
            # print "params", params            
            buf = struct.pack(self.byteorder + "4i" + str(len(msg)) + "f", *params)
        else:
            print "Message identifier unknown:  %s = %d, message: %s" % (arduino_msg_type_dict[msg_type], msg_type, msg)
            return   
           
        if buf != None:
            self.sock.send(buf)
            print "Sent message: %s to server " % (arduino_msg_type_dict[msg_type])

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
            return (msg_type, msg_counter) if msg_counter != -1 else (msg_type, None)
        elif msg_type == MSG_INFO:
            msg_counter = struct.unpack_from(self.byteorder + "i", raw_msg[:4])[0]
            msg_string = raw_msg[4:]
            return (msg_type, (msg_counter, msg_string))
        elif msg_type == MSG_CMD_GET_ENCODER_VAL:
            msg_counter = struct.unpack_from(self.byteorder + "i", raw_msg[:4])[0]
            encoder_value = struct.unpack_from(self.byteorder + "i", raw_msg[4:])[0]
            encoder_value_deg = self.calc_rel_enc_angle_degree(encoder_value)
            return (msg_type, (msg_counter, encoder_value_deg))
        else:
            print "%s: Message identifier unknown: %d, message: %s" % (self.parent.identifier, msg_type, raw_msg)
            return (False, False)
    
    #===========================================================================
    def __get(self, msg_type, msg = None, wait_for_response = True):
        self.set_msg_counter()
        self.set_state(BUSY)
        self.__send(msg_type, msg, wait_for_response)
        
        self.wait_flag = True # test, maybe this has to be changed
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
                    success = True #if msg else False
                except:
                    print "waiting for response"
                    time.sleep(0.01)
                    
                    if self.wait_flag == False: # ------> thread can be killed from outside event
                        return (msg_type, msg, success)
                    
        else: # just try to receive the cmd_received_message
            success = False
            try:
                msg_type, msg = self.__read()
                if msg_type == MSG_CMD_RECEIVED:
                    success = True
                    self.set_state(READY)
                    return (msg_type, msg, success)
                else:
                    return (msg_type, msg, success)
            except:
                return (msg_type, msg, success)
        
        self.set_state(READY)
        return (msg_type, msg, success)
    
    #===========================================================================
    def send_get_arduino_info(self):
        return self.__get(msg_type = MSG_INFO, msg = None, wait_for_response = True)
    
    #===========================================================================
    def send_get_encoder_values(self):
        return self.__get(msg_type = MSG_CMD_GET_ENCODER_VAL, msg = None, wait_for_response = True)
    
    
    # ===============================================================================================================================
    # motor angle calculations
    # ===============================================================================================================================
    def get_motor_pos_value_from_angle(self, angle_degrees, angle_per_step):
        '''returns the motor abs positions for an angle input
        please check: maybe needs to include a shift, depending on the zero position.
        '''
        #mapped_value = 8 * map_range(angle_degrees, -180, 180, -0.5 * 360 / angle_per_step, 0.5 * 360 / angle_per_step)
        mapped_value = map_range(angle_degrees, -180, 180, -0.5 * 360 / angle_per_step, 0.5 * 360 / angle_per_step)
        return mapped_value #* 48./40. #* -1.0 # be careful with the ratio
    #===========================================================================
    def get_angle_from_feed_len(self, feed_len, r):
        # feed len, r in mm
        angle = feed_len / r
        return math.degrees(angle)
    #===========================================================================
    def get_feed_len_from_angle(self, angle, r):
        #angle in deg, r in mm
        feed_len = r * math.radians(angle)
        return feed_len
    
    #===========================================================================
    def calc_rel_enc_angle_degree(self, encoder_pos):
        ticks_per_revolution = 4000 # 3600 for the new one
        return encoder_pos * 360. / (4. * ticks_per_revolution)
    
    #===========================================================================
    def calc_rel_enc_angle_rad(self, encoder_pos):
        ticks_per_revolution = 4000 # 3600 for the new one
        return encoder_pos * 2. * math.pi / (4. * ticks_per_revolution)
    
    """
    #===========================================================================
    def get_motor_pos_value_from_angle_old(self, angle_degrees):
        '''returns the motor abs positions for an angle input
        please check: maybe needs to include a shift, depending on the zero position.
        '''
        mapped_value = map_range(angle_degrees, -90, 90, -200, 200)
        return mapped_value * 34./29. """
    
    # ===============================================================================================================================
    # communcation with Arduino
    # ===============================================================================================================================
    
    #===========================================================================  
    def send_set_do(self, do = 0, state = 0, wait_for_response = True):
        """ command the arduino board to set a digital out to the given state. """
        return self.__get(msg_type = MSG_DIGITAL_OUT, msg = [do, state], wait_for_response = wait_for_response)
    
    #===========================================================================    
    def send_cmd_rotate_motor_abs(self, angle, motor_num, angle_per_step, wait_for_response = True):
        """ send a rotating command message to arduino with two int values. int1 = motortype, int2 = angle value """
        motor_pos = self.get_motor_pos_value_from_angle(angle, angle_per_step)
        return self.__get(msg_type = MSG_CMD_ROTATE_MOTOR_ABS, msg = [int(motor_pos), int(motor_num)], wait_for_response = wait_for_response)
    
    #===========================================================================    
    def send_cmd_rotate_motor_abs_pos(self, motor_pos, motor_num, wait_for_response = True):
        """ send a rotating command message to arduino with two int values. int1 = motortype, int2 = angle value """
        return self.__get(msg_type = MSG_CMD_ROTATE_MOTOR_ABS, msg = [int(motor_pos), int(motor_num)], wait_for_response = wait_for_response)
    
    #===========================================================================
    def send_cmd_rotate_motor_rel(self, angle, motor_num, angle_per_step, wait_for_response = True):
        """ send a rotating command message to arduino with two int values. int1 = motortype, int2 = angle value """
        motor_steps = self.get_motor_pos_value_from_angle(angle, angle_per_step)
        return self.__get(msg_type = MSG_CMD_ROTATE_MOTOR_REL, msg = [int(motor_steps), int(motor_num)], wait_for_response = wait_for_response)
    
    #===========================================================================
    # specific commands
    #===========================================================================
    
    #===========================================================================
    def send_cmd_bend(self, angle, overbending_value, wait_for_response = True):
        """ send a bend command message to arduino with two int values. int1 = motorsteps front, int2 = motorsteps back """
        angle_per_step_front = self.angle_per_step_m_front 
        motor_pos_front = self.get_motor_pos_value_from_angle(angle, angle_per_step_front)
        overbending_value = self.get_motor_pos_value_from_angle(overbending_value, angle_per_step_front)
        
        return self.__get(msg_type = MSG_CMD_BEND, msg = [int(motor_pos_front), int(overbending_value)], wait_for_response = wait_for_response)
    
    #===========================================================================
    def send_cmd_reset_encoder(self, wait_for_response = True):
        return self.__get(msg_type = MSG_CMD_RESET_ENCODER, msg = None, wait_for_response = wait_for_response)
    
    #===========================================================================
    def send_cmd_weld(self, wait_for_response = True):
        return self.__get(msg_type = MSG_CMD_WELD, msg = None, wait_for_response = wait_for_response)
    
    #===========================================================================
    def send_cmd_rotate_m_front_abs(self, angle, wait_for_response = True):
        motor_num = 1
        angle_per_step = self.angle_per_step_m_front
        return self.send_cmd_rotate_motor_abs(angle, motor_num, angle_per_step, wait_for_response = wait_for_response)
    
    #===========================================================================
    def send_cmd_rotate_m_front_rel(self, angle, wait_for_response = True):
        motor_num = 1
        angle_per_step = self.angle_per_step_m_front
        return self.send_cmd_rotate_motor_rel(angle, motor_num, angle_per_step, wait_for_response = wait_for_response)
    
    #===========================================================================
    def send_cmd_rotate_m_back_abs(self, angle, wait_for_response = True):
        motor_num = 2
        angle_per_step = self.angle_per_step_m_back
        return self.send_cmd_rotate_motor_abs(angle, motor_num, angle_per_step, wait_for_response = wait_for_response)
    
    #===========================================================================
    def send_cmd_rotate_m_back_rel(self, angle, wait_for_response = True):
        motor_num = 2
        angle_per_step = self.angle_per_step_m_back
        return self.send_cmd_rotate_motor_rel(angle, motor_num, angle_per_step, wait_for_response = wait_for_response)        
    
    # ==========================================================================
    def send_cmd_rotate_m_feeder_abs(self, angle, wait_for_response = True):
        motor_num = 3
        angle_per_step = self.angle_per_step_m_feeder
        return self.send_cmd_rotate_motor_abs(angle, motor_num, angle_per_step, wait_for_response = wait_for_response)
    
    #===========================================================================
    def send_cmd_rotate_m_feeder_rel(self, angle, wait_for_response = True):
        motor_num = 3
        angle_per_step = self.angle_per_step_m_feeder
        return self.send_cmd_rotate_motor_rel(angle, motor_num, angle_per_step, wait_for_response = wait_for_response)
    
    #===========================================================================
    def send_cmd_feed_wire_old(self, feed_len, wait_for_response = True):
        r = 23
        angle = self.get_angle_from_feed_len(feed_len, r)
        return self.send_cmd_rotate_m_feeder_rel(angle, wait_for_response = wait_for_response)
    
    #===========================================================================
    def send_cmd_feed_wire(self, feed_len, wait_for_response = True):
        
        r = 23
        angle_per_step = self.angle_per_step_m_feeder
        
        angle = self.get_angle_from_feed_len(feed_len, r)
        motor_steps = self.get_motor_pos_value_from_angle(angle, angle_per_step)
        
        return self.__get(msg_type = MSG_CMD_FEED_WIRE, msg = [motor_steps], wait_for_response = wait_for_response)
    
    #===========================================================================
    def send_cmd_eject_wire(self, wait_for_response = True):
        
        feed_len = -500
        
        r = 23
        angle_per_step = self.angle_per_step_m_feeder
        
        angle = self.get_angle_from_feed_len(feed_len, r)
        motor_steps = self.get_motor_pos_value_from_angle(angle, angle_per_step)
        
        return self.__get(msg_type = MSG_CMD_FEED_WIRE, msg = [motor_steps], wait_for_response = wait_for_response)
    
    #===========================================================================
    def send_cmd_load_wire(self, wait_for_response = True):
        
        feed_len = 500
        
        r = 23
        angle_per_step = self.angle_per_step_m_feeder
        
        angle = self.get_angle_from_feed_len(feed_len, r)
        motor_steps = self.get_motor_pos_value_from_angle(angle, angle_per_step)
        
        return self.__get(msg_type = MSG_CMD_FEED_WIRE, msg = [motor_steps], wait_for_response = wait_for_response)
    
    # int1 = 0: OPEN # int1 = 1: CLOSE # int1 = 1: STOP
    #===========================================================================
    def send_electrodes_open(self, wait_for_response = True):
        return self.__get(msg_type = MSG_CMD_ELECTRODES, msg = [0], wait_for_response = wait_for_response)
    
    #===========================================================================
    def send_electrodes_close(self, wait_for_response = True):
        return self.__get(msg_type = MSG_CMD_ELECTRODES, msg = [1], wait_for_response = wait_for_response)
    
    #===========================================================================
    def send_electrodes_stop(self, wait_for_response = True):
        return self.__get(msg_type = MSG_CMD_ELECTRODES, msg = [2], wait_for_response = wait_for_response)
    
    
    # int1 = 0: MOVE_TO_BACK, int1 = 1: MOVE_TO_FRONT, int1 = 2: CUT, int1 = 3: RELEASE
    #===========================================================================
    def send_nippers_move_to_back(self, wait_for_response = True):
        return self.__get(msg_type = MSG_CMD_NIPPERS, msg = [0], wait_for_response = wait_for_response)
    
    #===========================================================================
    def send_nippers_move_to_front(self, wait_for_response = True):
        return self.__get(msg_type = MSG_CMD_NIPPERS, msg = [1], wait_for_response = wait_for_response)
    
    #===========================================================================
    def send_nippers_cut(self, wait_for_response = True):
        return self.__get(msg_type = MSG_CMD_NIPPERS, msg = [2], wait_for_response = wait_for_response)
    
    #===========================================================================
    def send_nippers_release(self, wait_for_response = True):
        return self.__get(msg_type = MSG_CMD_NIPPERS, msg = [3], wait_for_response = wait_for_response)
    
    
    #===========================================================================
    def send_cmd_feed_cut_weld(self, feed_len, wait_for_response = True):
        
        r = 23
        angle_per_step = self.angle_per_step_m_feeder
        
        angle = self.get_angle_from_feed_len(feed_len, r)
        motor_steps = self.get_motor_pos_value_from_angle(angle, angle_per_step)
        
        return self.__get(msg_type = MSG_CMD_ROUTINE_FEED_CUT_WELD, msg = [motor_steps], wait_for_response = wait_for_response)

    
         

if __name__ == "__main__":
    aclient = ArduinoClient(host = "192.168.10.177", port=30001)
    time.sleep(0.5)
    
    aclient.connect()
    time.sleep(2.5)
    print "Connection: ", aclient.connected
    v1 = 0
    v2 = 50
    for i in range(1000):
        
        f = v1 if i%2==0 else v2
        #msg_type, msg_counter, success = aclient.send_cmd_bending(f=f)
        msg_type, msg, success = aclient.send_get_arduino_info()
        print "RECEIVED MSG FROM ARDUINO: "
        print "MSG TYPE: ", arduino_msg_type_dict[msg_type]
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