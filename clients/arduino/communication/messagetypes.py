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

#===============================================================================
# MESSAGE TYPES ARDUINO
#===============================================================================

""" Message Types for sending and receiving messages from the clients
Message always consists of uint4: length of message, uint4: type of message, message (depends on the specific kind) """ 


MSG_CMD_RECEIVED = 1 # empty
MSG_CMD_EXECUTED = 2 # empty
MSG_DIGITAL_OUT = 3 # [int do, boolean]
MSG_FLOAT_LIST = 4 # [float, float, ...]
MSG_INFO = 5
MSG_STRING = 6
MSG_CMD_ROTATE_MOTOR_ABS = 7 # [int1, int2] # int1 = absolute motor value for rotation, int2 = motor front or motor back
MSG_CMD_ROTATE_MOTOR_REL = 8 # [int1, int2] # int1 = absolute motor value for rotation, int2 = motor front or motor back
MSG_CMD_FEED_WIRE = 9 # [int1] # int1 = feed len in mm
MSG_CMD_ELECTRODES = 10 # [int1] # int1 = 0: OPEN, # int1 = 1: CLOSE, # int1 = 1: STOP
MSG_CMD_NIPPERS = 11 # int1 = 0: MOVE_TO_BACK, int1 = 1: MOVE_TO_FRONT, int1 = 2: CUT, int1 = 3: RELEASE
MSG_CMD_WELD = 12 # [] empty message
MSG_CMD_ROUTINE_FEED_CUT_WELD = 13 # [int1] # int1 = feed len in mm
MSG_CMD_BEND = 14 # [int1, int2]  # int1 = abs motor value for rot front, # int2 = abs motor value for rot back --> for now: rotates both  motors at the same time
MSG_CMD_RESET_ENCODER = 15 # [] empty message
MSG_CMD_GET_ENCODER_VAL = 16 # [] empty message, Arduino returns one long value

arduino_msg_type_dict = {1:"MSG_CMD_RECEIVED", 2:"MSG_CMD_EXECUTED", 3:"MSG_DIGITAL_OUT", \
                       4:"MSG_FLOAT_LIST", 5:"MSG_INFO", 6:"MSG_STRING", \
                       7:"MSG_CMD_ROTATE_MOTOR_ABS", 8:"MSG_CMD_ROTATE_MOTOR_REL", \
                       9:"MSG_CMD_FEED_WIRE", 10:"MSG_CMD_ELECTRODES", \
                       11:"MSG_CMD_NIPPERS", 12:"MSG_CMD_WELD", 13:"MSG_CMD_ROUTINE_FEED_CUT_WELD" , \
                       14:"MSG_CMD_BEND", 15:"MSG_CMD_RESET_ENCODER", 16:"MSG_CMD_GET_ENCODER_VAL"}