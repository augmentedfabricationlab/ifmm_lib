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

#===============================================================================
# MESSAGE TYPES VISION
#===============================================================================

""" Message Identifiers for sending and receiving messages from the BASE only
Message always consists of [Length[just message body in {}, 4 byte int], MsgType[4byte int], {counter[4byte int], timestamp[8byte double], msg[length depending]}] """ 

MSG_INVALID = 0

# WIRE LINE ESTIMATION
MSG_VISION_GET_CURRENT_WIRE_LINE = 1 # msg is empty
MSG_VISION_CURRENT_WIRE_LINE = 2 # client returns [x_p1, y_p1, z_p1, x_p2, y_p2, z_p2]
MSG_VISION_GET_CURRENT_WIRE_LINE_1LINEINPUT = 3 # msg is [x_p1, y_p1, z_p1, x_p2, y_p2, z_p2] = expected line from and line to in camera frame
MSG_VISION_CURRENT_WIRE_LINE_1LINEOUTPUT = 4 # client returns [x_m1, y_m1, z_m1, x_v1, y_v1, z_v1] = midpoint of estimated wire (= intersection of normal plane), vector of the wire direction unified
MSG_VISION_GET_CURRENT_WIRE_LINE_3LINEINPUT = 5 # msg is [estimation_type, x_p1_l1, y_p1_l1, z_p1_l1, x_p2_l1, y_p2_l1, z_p2_l1 * 3] = expected 3 lines, continuous element and two discrete elements, line from and line to in camera frame, int est_type tells which one to match
MSG_VISION_CURRENT_WIRE_LINE_3LINEOUTPUT = 6 # client returns [x_m1, y_m1, z_m1, x_v1, y_v1, z_v1 * 2] = midpoint of estimated wire (= intersection of normal plane), vector of the wire direction unified, startpoint of the discrete wire element, vector of the discrete wire element direction unified

# CAMERA CALIBRATION
MSG_VISION_SEND_CURRENT_ARM_POSE = 7  # send msg is the current pose of the arm [x, y, z, qw, qx, qy, qz] --> client returns msg MSG_VISION_SUCCESS (empty msg)

# GENERIC
MSG_VISION_SEND_SUCCESS = 8 # server sends empty msg (as OK!)
MSG_VISION_SUCCESS = 9 # client returns empty msg (as OK!)

# BASE POSE ESTIMATION
MSG_VISION_GET_CURRENT_BASE_POSE = 10 # msg ist empty
MSG_VISION_CURRENT_BASE_POSE = 11 # client returns [x, y, z, qw, qx, qy, qz]

# BASE POSE ESTIMATION 2
MSG_VISION_GET_CURRENT_BASE_POSE_2 = 12 # [x, y, z, qw, qx, qy, qz] --> pose of T_R_E
MSG_VISION_CURRENT_BASE_POSE_2 = 13 # client returns [x, y, z, qw, qx, qy, qz]


vision_msg_type_dict = {0:"MSG_INVALID", 1:"MSG_VISION_GET_CURRENT_WIRE_LINE", 2:"MSG_VISION_CURRENT_WIRE_LINE", \
                      3:"MSG_VISION_GET_CURRENT_WIRE_LINE_1LINEINPUT", 4:"MSG_VISION_CURRENT_WIRE_LINE_1LINEOUTPUT", \
                      5:"MSG_VISION_GET_CURRENT_WIRE_LINE_3LINEINPUT", 6:"MSG_VISION_CURRENT_WIRE_LINE_3LINEOUTPUT", \
                      7:"MSG_VISION_SEND_CURRENT_ARM_POSE", 8:"MSG_VISION_SEND_SUCCESS", 9:"MSG_VISION_SUCCESS", \
                      10:"MSG_VISION_GET_CURRENT_BASE_POSE", 11:"MSG_VISION_CURRENT_BASE_POSE", \
                      12:"MSG_VISION_CURRENT_BASE_POSE_2", 13:"MSG_VISION_CURRENT_BASE_POSE_2"}







