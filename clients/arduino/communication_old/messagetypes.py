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

MSG_INVALID = 0

MSG_CMD_RECEIVED = 1 # empty
MSG_CMD_EXECUTED = 2 # empty
MSG_CMD_BEND = 3 # [float]
MSG_CMD_INSERT_VERTICAL = 4 # [float]
MSG_CMD_RETRACT = 5 # [float]
MSG_DIGITAL_OUT = 6 # [int do, boolean]
MSG_FLOAT_LIST = 7 # [float, float, ...]
MSG_STOP = 8
MSG_INFO = 9
MSG_STRING = 10
MSG_CMD_INSERT_AND_BEND = 11 # [int1, int2] # int1 = long or short vertical boolean, int2 = absolute motor value for bending
MSG_CMD_ROTATE_MOTOR_ABS = 12 # [int1, int2] # int1 = absolute motor value for rotation, int2 = motor front or motor back
MSG_CMD_ROTATE_MOTOR_REL = 13 # [int1, int2] # int1 = rel motor value for rotation, int2 = motor front or motor back

MSG_CMD_INSERT_VERTICAL_BEF_TILT = 14; #[float] # = long or short vertical
MSG_CMD_BEND_AFTER_TILT = 15; #[float] # = bending angle

msg_types_str_array = ["MSG_INVALID", "MSG_CMD_RECEIVED", "MSG_CMD_EXECUTED", "MSG_CMD_BEND", \
                       "MSG_CMD_INSERT_VERTICAL", "MSG_CMD_RETRACT", "MSG_DIGITAL_OUT", "MSG_FLOAT_LIST", \
                       "MSG_STOP", "MSG_INFO", "MSG_STRING", "MSG_CMD_INSERT_AND_BEND", "MSG_CMD_ROTATE_MOTOR_ABS", \
                       "MSG_CMD_ROTATE_MOTOR_REL", "MSG_CMD_INSERT_VERTICAL_BEF_TILT", "MSG_CMD_BEND_AFTER_TILT"]