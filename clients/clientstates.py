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

" Robot arm client states: "
STATE_READY = 1 # the buffer of the robot is empty, he is ready to receive commands (number = stacksize)
STATE_EXECUTING = 2 # the robot is executing the command
STATE_READY_TO_RECEIVE = 3 # the buffer of the robot has space, he is ready to receive the next command
STATE_COMMAND_EXECUTED = 6

" other clients states: "
READY = 4
BUSY = 5

states_dict = {1:"STATE_READY", 2:"STATE_EXECUTING", 3:"STATE_READY_TO_RECEIVE", 4: "READY", 5:"BUSY", 6:"STATE_COMMAND_EXECUTED"}