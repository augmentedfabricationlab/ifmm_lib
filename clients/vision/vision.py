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

Created on 30.01.2017

@author: kathrind
'''

from ifmm_lib.clients.vision.communication.client_vision import ClientVision
from ifmm_lib.clients.vision.communication.messagetypes import vision_msg_type_dict

import time
import math

from ifmm_lib.geometry import Frame
#import Rhino.Geometry as rg


class Vision(ClientVision):
    """ ...
    
    """
    
    def __init__(self, host='127.0.0.1', port=20003):
        ClientVision.__init__(self, host, port)  

        self.current_line_est_values = None

if __name__ == '__main__':
    
    vals = [0.023329609478325467, -0.037088400300775336, 0.19927068735906767, 0.049659037470244104, -0.0084515710682616721, 0.15175112938846383, 0.023329609478325467, -0.037088400300775336, 0.19927068735906767, 0.023458227464983109, -0.018044290241578154, 0.21140184119950789, 0.049659037470244104, -0.0084515710682616721, 0.15175112938846383, 0.049608029208620112, 0.010617040427573783, 0.16385811750818341]
    print len(vals) 
    
    vision_client = Vision(host='192.168.125.50', port=20003)
    print vision_client.current_line_est_values
    print vision_client.connected
    print vision_client.host
    
    
    print "VisionClient created."
    
    vision_client.connect()
    time.sleep(0.5)
    print "Connection: ", vision_client.connected
    
    
    
    for i in range(3):
        msg_type, msg, success = vision_client.get_current_base_pose()
        print "RECEIVED MSG FROM BASE: "
        print "MSG TYPE: ", vision_msg_type_dict[msg_type] #vision_msg_types_str_array[msg_type]
        print "MSG: ", msg
        print "SUCCESS" if success==True else "NO SUCCESS" 
        time.sleep(0.1)
        
       

        msg_type, msg, success = vision_client.get_current_line_estimation_3lineinput(vals)
        
        if success:
            print "RECEIVED MSG FROM BASE: "
            print "MSG TYPE: ", vision_msg_type_dict[msg_type]
            print "MSG: ", msg
            print "SUCCESS" if success==True else "NO SUCCESS"
            vision_client.current_line_est_values = msg
        else:
            print "MSG WAS NOT RECEIVED"
            vision_client.current_line_est_values = None
    
        print "TVISION CONNECTION STATE: ", vision_client.connected
        print "TVISION CURRENT WIRE LINE: ", vision_client.current_line_est_values
        
    """
    for i in range(3):
        msg_type, msg, success = vision_client.send_current_arm_pose([0.11626742449272023, -1.1021150988729778, 0.11692848402380923, -0.12930746011595484, 0.38686473260011522, 0.90563749640245828, 0.11591369435269336])
        print "RECEIVED MSG FROM BASE: "
        print "MSG TYPE: ", vision_msg_types_str_array[msg_type]
        print "MSG: ", msg
        print "SUCCESS" if success==True else "NO SUCCESS"
        
        if not success:
            vision_client.close()
            time.sleep(0.1)
            vision_client.connect()
            
        #print "MSG COUNTER: ", msg_counter
        time.sleep(1)"""
        
    """
    for i in range(3):
        msg_type, msg, success = vision_client.get_current_wire_line()
        print "RECEIVED MSG FROM BASE: "
        print "MSG TYPE: ", vision_msg_types_str_array[msg_type]
        print "MSG: ", msg
        print "SUCCESS" if success==True else "NO SUCCESS"
        
        if not success:
            vision_client.close()
            time.sleep(0.1)
            vision_client.connect()
            
        #print "MSG COUNTER: ", msg_counter
        time.sleep(1)"""
        
    vision_client.close()
    print "Connection: ", vision_client.connected
    

