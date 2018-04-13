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

Created on 02.11.2016

@author: kathrind
'''

from ifmm_lib.geometry import Frame

from client_container import ClientContainer
from messages.messagetypes import MSG_CURRENT_POSE_CARTESIAN, MSG_CURRENT_POSE_JOINT, MSG_COMMAND, MSG_STOP
from messages.messagetypes import CMD_GO_TO_TASKTARGET, CMD_GO_TO_JOINTTARGET_ABS, CMD_GO_TO_JOINTTARGET_REL, CMD_PICK_BRICK, CMD_PLACE_BRICK, CMD_PICK_BRICK_FROM_POSE

import time

import Rhino.Geometry as rg
import math as m



class ABBCommunication(ClientContainer):
    """ The class ABBComm extends the Clientcontainer class. 
    It can send and receive data from the ABB arm
    
    """
    
    def __init__(self, identifier, host='127.0.0.1', port_snd=30003, port_rcv=30004, ghenv = None):
        ClientContainer.__init__(self,  identifier, host, port_snd, port_rcv, ghenv = ghenv)
        
        " create "
        self.tool_frame = Frame(draw_geo = True)
        
        # init values for command messages
        self.int_speed = 0 # speed: 0 = slow, 1 = mid, 2 = fast
        self.float_duration = 0 #duration is not used, use velocity instead
        self.int_zonedata = 10 # zonedata: in mm
        self.int_tool = 1 # toolnumber: 0 = tool0, 1 = mesh mould tool, 2 = mesh mould tool with bending pin, 3 = mtip
        self.float_arbitrary = 0 #spare field for any kind of input
        
        # home positions cartesian
        self.tool_plane_home_mid = rg.Plane(rg.Point3d(1000,0,650), rg.Vector3d(1,0,0), rg.Vector3d(0,-1,0))
        self.tool_plane_home_left = rg.Plane(rg.Point3d(-665.507, 629.086, 720.0), rg.Vector3d(-1,0,0), rg.Vector3d(0,1,0))
        
        # home positions joint targets
        self.jointtarget_home_zero = [0,0,0,0,0,0]
        self.jointtarget_home_pickbrick = [130.9, -56.8, 58.3, 7.4, 30.1, 51.9]
        
        self.current_joint_values = [0,0,0,0,0,0]
        self.current_tool0_pose = [0,0,0,0,0,0,0]
        #self.ghComp = None

    # =================================================================================    
    # robot tool
    # =================================================================================     
    # =================================================================================        
    def set_tool_to_plane(self, plane):
        """ move the base to a plane position """
        self.tool_frame.set_to_plane(plane)
    # =================================================================================        
    def get_tool_plane(self):
        """ return the plane of the baseframe """
        return self.tool_frame.plane
    # =================================================================================    
    def get_tool_tmatrix_from(self):
        """ return the transformation matrix (Transform class of Rhino) of the baseframe to the origin """
        return self.tool_frame.get_tmatrix_from()
    # =================================================================================    
    def get_tool_tmatrix_to(self):
        """ return the transformation matrix (Transform class of Rhino) of the origin to the baseframe """
        return self.tool_frame.get_tmatrix_to()
    
    # =================================================================================    
    # receive robot info from queues
    # =================================================================================    
    # =================================================================================               
    def get_current_pose_cartesian(self):
        """ get the current tool pose from the queue and set the tool_frame according to the pose """
        msg_current_pose_cart = self.get_from_rcv_queue(MSG_CURRENT_POSE_CARTESIAN)
        if msg_current_pose_cart <> None:
            pose = msg_current_pose_cart[1]
            self.current_tool0_pose = pose
            #self.tool_frame.set_to_pose(pose)
            return pose
        else: 
            return None 
        
    # =================================================================================               
    def get_current_pose_joint(self):
        """ get the current tool pose from the queue and set the tool_frame according to the pose """
        msg_current_pose_joint = self.get_from_rcv_queue(MSG_CURRENT_POSE_JOINT)
        if msg_current_pose_joint <> None:
            pose_joint = msg_current_pose_joint[1]
            self.current_joint_values = [m.degrees(pj) for pj in pose_joint]
            return pose_joint
        else: 
            return None
    
    # =================================================================================    
    # send robot commands
    # =================================================================================
    # =================================================================================           
    def send_stop(self):
        """ send stop to robot """
        #TODO: implement on Rapid side
        self.send(MSG_STOP)
            
    # =================================================================================       
    def send_pose_cartesian(self, plane, int_arr=None):
        """ create command from plane and send task target to robot, 
        int_arr can be defined outside, or if None, default values are sent. 
        int_arr = [int_speed, float_duration, int_zonedata, int_tool, float_arbitrary] """
        
        frame = Frame(plane)
        pose = frame.get_pose_quaternion() # pose = [x,y,z,qw,qx,qy,qz]
        if int_arr == None:
            cmd = [CMD_GO_TO_TASKTARGET] + pose + [self.int_speed, self.float_duration, self.int_zonedata, self.int_tool, self.float_arbitrary]
        else:
            cmd = [CMD_GO_TO_TASKTARGET] + pose + int_arr
        
        self.send(MSG_COMMAND, cmd)
        
    # =================================================================================           
    def send_pose_cartesian_list(self, planes, int_arr=None):
        """ create command from planes and send task targets to robot, 
        int_arr can be defined outside, or if None, default values are sent. 
        int_arr = [int_speed, float_duration, int_zonedata, int_tool, float_arbitrary] """   
        for plane in planes:
            self.send_pose_cartesian(plane, int_arr)
    
        # =================================================================================           
    def send_axes_relative(self, axes, int_arr=None):
        """ relative joint axes commands for the abb arm """
        if int_arr:
            cmd = [CMD_GO_TO_JOINTTARGET_REL] + axes + [0] + int_arr
        else:
            cmd = [CMD_GO_TO_JOINTTARGET_REL] + axes + [0] + [self.int_speed, self.float_duration, self.int_zonedata, self.int_tool, 1]
        self.send(MSG_COMMAND, cmd)
    
    # =================================================================================           
    def send_axes_absolute(self, axes, int_arr=None):
        """ absolute joint axes commands for the abb arm """   
        if int_arr:
            cmd = [CMD_GO_TO_JOINTTARGET_ABS] + axes + [0] + int_arr
        else:
            cmd = [CMD_GO_TO_JOINTTARGET_ABS] + axes + [0] + [self.int_speed, self.float_duration, self.int_zonedata, self.int_tool, 1]
        self.send(MSG_COMMAND, cmd) 
    
    # =================================================================================           
    def send_pose_cartesian_home(self, int_arr=None):
        """ send the "home" position as task target as defined in init """
        self.send_pose_cartesian(self.tool_plane_home_mid, int_arr)
        
    # =================================================================================           
    def send_pose_joint_home(self, int_arr=None):
        """ send the "home" position as jointtarget as defined in init """
        self.send_axes_absolute(self.jointtarget_home_zero, int_arr)
    
    # =================================================================================           
    def send_pose_joint_pick_brick(self, int_arr=None):
        """ send the "home" position as jointtarget as defined in init """
        self.send_axes_absolute(self.jointtarget_home_pickbrick, int_arr)
        
    # =================================================================================               
    def send_pick_brick_from_pose(self, plane, int_arr=None):
        """ send a command for picking up the material and go to the given plane 
        sequence: 
        1. drive from actual pos to pick up the brick at the given plane
        (== >> this routine is defined in RobotStudio, the command consist only out of the pick-up plane.
        """    
        frame = Frame(plane)
        pose = frame.get_pose_quaternion() # pose = [x,y,z,qw,qx,qy,qz]
        if int_arr == None:
            cmd = [CMD_PICK_BRICK_FROM_POSE] + pose + [self.int_speed, self.float_duration, self.int_zonedata, self.int_tool, self.float_arbitrary]
        else:
            cmd = [CMD_PICK_BRICK_FROM_POSE] + pose + int_arr
        self.send(MSG_COMMAND, cmd)
        
    # =================================================================================               
    def send_pick_brick_from_feed(self, fullbrick = True, int_arr=None):
        """ send a command for picking up the material and go to the given plane sequence: 
        1. drive from actual pos to the homeposition, defined as jointtarget, on the left side of the robot.
        2. go on the trajectory to pick up a brick from the feeder and drive back to the homeposition
        == >> this routine is defined in RobotStudio, the command consist only out of the homeposition.
        cmd = [CMD_PICK_BRICK, joints, int_speed, float_duration, int_zonedata, int_tool, float_arbitrary] float_arbitrary => 0 = fullbrick  / 1 = halfbrick
        """ 
        
        jointpose = self.jointtarget_home_pickbrick 
        float_arbitrary = 0 if fullbrick == True else 1 #this parameter has to read in Rapid, to either pick a fullbrick or a halfbrick
        
        if int_arr == None:
            cmd = [CMD_PICK_BRICK] + jointpose + [0] + [self.int_speed, self.float_duration, self.int_zonedata, self.int_tool, float_arbitrary]
        else:
            cmd = [CMD_PICK_BRICK] + jointpose + [0] + int_arr
        
        self.send(MSG_COMMAND, cmd)

    # =================================================================================
    def send_place_brick(self, plane, send_pick_brick_from_feed = True, fullbrick = True, int_arr = None):
        """ send a command for placing the material at the given plane 
        sequence: 
        1. drive to home_pos, pick a brick and drive back to homepos
        1. drive from home pos to the a point 20 cm above the given plane
        2. go on the trajectory to place the brick and drive back to a point 20 cm above the given plane
        == >> this routine is defined in RobotStudio, and is defined 
        cmd = [x,y,z,q1,q2,q3,q4,int_speed, int_zonedata, int_proc] int_proc = 3 = place the brick
        
        cmd = [CMD_PICK_BRICK, robtarget, int_speed, float_duration, int_zonedata, int_tool, float_arbitrary]
        """
        if send_pick_brick_from_feed:
            self.send_pick_brick_from_feed(fullbrick, int_arr)
        
        frame = Frame(plane)
        pose = frame.get_pose_quaternion() # pose = [x,y,z,qw,qx,qy,qz]
        
        if int_arr == None:
            cmd = [CMD_PLACE_BRICK] + pose + [self.int_speed, self.float_duration, self.int_zonedata, self.int_tool, self.float_arbitrary]
        else:
            cmd = [CMD_PLACE_BRICK] + pose + int_arr
        
        self.send(MSG_COMMAND, cmd)

    # =================================================================================
    def set_home_pos_left_to_origin(self):
        self.tool_plane_home_left = rg.Plane(rg.Point3d(-665.507, 629.086, 720.0), rg.Vector3d(-1,0,0), rg.Vector3d(0,1,0))
        
    # =================================================================================
    def get_home_pos_left(self):
        return self.tool_plane_home_left
    
    # =================================================================================
    def get_home_pos_left_for_180(self):
        plane = rg.Plane(self.get_home_pos_left())
        plane.Translate(rg.Vector3d(1200,0,0))
        plane.Rotate(m.radians(180), plane.ZAxis, plane.Origin)
        return plane
    
    # =================================================================================    
    # set command parameters (for int_arr)
    # =================================================================================   
    # =================================================================================
    def set_speed_fast(self):
        self.int_speed = 2
        self.int_zonedata = 100
    # =================================================================================
    def set_speed_mid(self):
        self.int_speed = 1
        self.int_zonedata = 20
    # =================================================================================
    def set_speed_slow(self):
        self.int_speed = 0
        self.int_zonedata = 10 
    # =================================================================================
    def set_tool0(self):
        # toolnumber: 0 = tool0, 1 = vacgrip_vert, 2 = hokuyo
        self.int_tool = 0
    # =================================================================================
    def set_tool_meshmould(self):
        # toolnumber: 0 = tool0, 1 = meshmould, 2 = hokuyo
        self.int_tool = 1
    # =================================================================================
    def set_tool_meshmould_withbendingpin(self):
        # toolnumber: 0 = tool0, 1 = meshmould, 2 = hokuyo
        self.int_tool = 2
    # =================================================================================
    def set_tool_mtip(self):
        # toolnumber: 0 = tool0, 1 = meshmould, 2 = hokuyo
        self.int_tool = 3
    
    """
    # =================================================================================
    def set_tool_vakgrip(self):
        # toolnumber: 0 = tool0, 1 = vacgrip_vert, 2 = hokuyo
        self.int_tool = 1
    """
    """
    # =================================================================================
    def set_tool_concrete_surface(self, ttype="puncher"):
        # toolnumber: 0 = tool0, 1 = meshmould, 2 = hokuyo, 3 = puncher, 4 = roller, 5 = sweeper
        if ttype == "puncher":
            self.int_tool = 4
        elif ttype == "roller":
            self.int_tool = 5
        else: # ttype = "sweeper"
            self.int_tool = 6
    """
        
    
if __name__ == '__main__':
    robot = ABBCommunication("ABB", '192.168.125.1')
    robot.start()
    time.sleep(1)
    
    robot.set_speed_slow()
    print robot.get_state()
    
    robot.send_pose_joint_pick_brick()
    
    time.sleep(0.1)
    print robot.get_state()
    
    while robot.get_state() <> 1:
        
        #robot.send_pose_joint_home()
        #time.sleep(0.2)
        #robot.send_pose_joints_pick_brick()
        time.sleep(0.01)
        current_pose_joints = robot.get_current_pose_joint()
        print current_pose_joints
        print robot.cmd_exec_counter_from_client
        print robot.cmd_counter_to_client
    
    print "comd sent: ", robot.cmd_counter_to_client
    print "cmd exec: ", robot.cmd_exec_counter_from_client
    print "state: ", robot.get_state()
    """
    robot.send_pose_joint_home()
    
    
    time.sleep(0.1)
    
    print robot.get_state()
    
    while robot.get_state() <> 1:
        
        #robot.send_pose_joint_home()
        #time.sleep(0.2)
        #robot.send_pose_joints_pick_brick()
        time.sleep(0.01)
        current_pose_joints = robot.get_current_pose_joint()
        print current_pose_joints
    
    robot.wait_for_state_ready(0.0)   
        
    robot.set_speed_slow()"""
    
    print "ready"
    robot.close()
    print "closed"
