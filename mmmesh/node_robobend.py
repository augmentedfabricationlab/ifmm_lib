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

Created on 02.11.2016

@author: kathrind
'''

import Rhino.Geometry as rg
import ghpythonlib.components as ghcomp
import math as m
import random

class Node:
    
    def __init__(self, node_type, layer_type, index_row, index_layer_list):
        
        self.node_type = node_type # can be "exterior_srf1", "exterior_srf2", "interior"
        self.layer_type = layer_type
        
        self.index_row = index_row
        self.index_col = None 
        self.index = None
        self.index_layer_list = index_layer_list #position in one layer
        
        self.is_built = False
        
        self.plane_wrist = None # is also the plane to drive to, if no discrete element is inserted
        #self.wrist_plane_for_inserting_tilted = None
        self.plane_for_inserting = None
        self.plane_for_bending = None
        
        self.absolute_bending_angle = 0
        
        self.axis = None # vector of continuous element
        self.line_continuous_element = None # line of the element in the continuousdirection (= vertical direction)
        
        self.insert_discrete_element = False #if node has a discrete element to insert
        self.line_discrete_element = None
        
        self.intersect_line_cont_disc = None
        
        self.collision = False
        self.collision_geo = None
        
        # for the line estimation by the endeffector cameras
        self.neighbor_node_for_est_01 = None 
        self.neighbor_node_for_est_02 = None
        
        #self.estimated_lines_in_cam_frame = []
        self.lines_for_estimation_WCS = []
        
        self.estimate_with_eeff_cam = False
        self.estimation_type = 0 # 1: front discrete wire, 2: back discrete wire, 3: match both wires
        
        self.estimated_lines_CCS = []
        self.estimated_lines_WCS = []
        
        ### for comparison purposes
        self.estimated_lines_WCS_after_corr = []
        
        ### just for testing purposes
        #self.base_state_estimation_plane = None
        
        #self.estimated_lines_in_cam_frame_mtip = []
        #self.estimated_lines_global_mtip = []
        
        #self.estimated_lines_global_before_corr = []
        #self.estimated_lines_global_after_corr = []
        #self.T_est_exp = None
        
        self.color = ghcomp.ColourRGB(100, 0, 0, 0)
        
        self.corr_val = 0
        self.plane_for_inserting_corr = None
        self.plane_for_bending_corr = None
    
    def set_built_state(self, built_state):
        self.is_built = built_state
    
    def set_index(self, index):
        self.index = index
    
    def set_index_col(self, index_col):
        self.index_col = index_col
    
    def get_line_geo_with_colors(self):
        geo = []
        colors = []
        if self.line_discrete_element: 
            geo.append(self.line_discrete_element)
            colors.append(self.color)
        if self.line_continuous_element: 
            geo.append(self.line_continuous_element)
            colors.append(self.color)
        
        return geo, colors
    
    def get_line_geo(self):        
        if self.line_discrete_element and self.line_continuous_element:
            return [self.line_discrete_element, self.line_continuous_element]
        elif self.line_discrete_element and self.line_continuous_element == None:
            return [self.line_discrete_element]
        elif self.line_discrete_element == None and self.line_continuous_element:
            return [self.line_continuous_element]
        else:
            return []
        
    def fill_params(self, plane_wrist, axis = None, insert_discrete_element = False, plane_for_inserting = None, line_discrete_element = None, intersect_line_cont_disc = None):         
        self.plane_wrist = plane_wrist
        self.axis = axis
        self.insert_discrete_element = insert_discrete_element
        self.plane_for_inserting = plane_for_inserting  
        self.line_discrete_element = line_discrete_element
        self.intersect_line_cont_disc = intersect_line_cont_disc
    
    def get_lines_for_estimation_CCS(self, cam_plane_trfd_global, return_as_lines = False):
        ''' this method returns the lines for the line estimation with the endeffector cameras in the camera frame
        if return_as_lines = True > return the data as lines, otherwise as a list of 3*6 (= 18) floats
        line 1 = line_c = line from discrete element 01 startpt to discrete element 02 startpt (= continuous element)
        line 2 = line_d1 = line of discrete element 01 (= discrete element in front)
        line 3 = line_d2 = line of discrete element 02 (= discrete element in the back)
        '''
        
        lines_for_est_WCS = self.lines_for_estimation_WCS #[line_c, line_d_front, line_d_back]
        line_c_WCS, line_d_front_WCS, line_d_back_WCS = lines_for_est_WCS
        
        T_from_world_to_cam = rg.Transform.ChangeBasis(rg.Plane.WorldXY, cam_plane_trfd_global)
        lines_for_est_in_cam_frame = ghcomp.Transform(lines_for_est_WCS, T_from_world_to_cam)
        
        if return_as_lines:
            return lines_for_est_in_cam_frame
        
        else:
            
            if line_d_front_WCS and line_d_back_WCS:
                #node.estimation_type = 2
            
                l1, l2, l3 = lines_for_est_in_cam_frame
                l1_values = [l1.FromX, l1.FromY, l1.FromZ, l1.ToX, l1.ToY, l1.ToZ] 
                l2_values = [l2.FromX, l2.FromY, l2.FromZ, l2.ToX, l2.ToY, l2.ToZ] #if l2 else [-1 for i in range(6)]
                l3_values = [l3.FromX, l3.FromY, l3.FromZ, l3.ToX, l3.ToY, l3.ToZ] #if l3 else [-1 for i in range(6)]
                l = l1_values + l2_values + l3_values
            
            elif line_d_front_WCS and line_d_back_WCS == None:
                #node.estimation_type = 1
                lc, ld = ghcomp.Transform([line_c_WCS, line_d_front_WCS], T_from_world_to_cam)
                lc_values = [lc.FromX, lc.FromY, lc.FromZ, lc.ToX, lc.ToY, lc.ToZ] 
                ld_values = [ld.FromX, ld.FromY, ld.FromZ, ld.ToX, ld.ToY, ld.ToZ]
                lempty_values = [-1 for i in range(6)]
                
                l = lc_values + ld_values + lempty_values

                
            elif line_d_front_WCS == None and line_d_back_WCS:
                #node.estimation_type = 27
                
                lc, ld = ghcomp.Transform([line_c_WCS, line_d_back_WCS], T_from_world_to_cam)
                lc_values = [lc.FromX, lc.FromY, lc.FromZ, lc.ToX, lc.ToY, lc.ToZ] 
                ld_values = [ld.FromX, ld.FromY, ld.FromZ, ld.ToX, ld.ToY, ld.ToZ]
                lempty_values = [-1 for i in range(6)]
                
                l = lc_values + lempty_values + ld_values
            
            return [v/1000 for v in l]
    
    def get_sim_est_values_CCS(self, cam_plane_trfd_WCS, rval = 0.1, with_random = True):
        """ just for testing the calculation of the line estimation"""
        
        line_c_CCS, line_d_front_CCS, line_d_back_CCS = self.get_lines_for_estimation_CCS(cam_plane_trfd_WCS, return_as_lines = True)
        
        m0_x, m0_y, m0_z = (line_c_CCS.PointAt(0.5).X, line_c_CCS.PointAt(0.5).Y, line_c_CCS.PointAt(0.5).Z)
        v0 = line_c_CCS.Direction
        v0.Unitize()
        v0_x, v0_y, v0_z = v0
        
        if line_d_back_CCS:
            m1_x, m1_y, m1_z = (line_d_back_CCS.PointAt(0.5).X, line_d_back_CCS.PointAt(0.5).Y, line_d_back_CCS.PointAt(0.5).Z)
            v1 = line_d_back_CCS.Direction
        else:
            m1_x, m1_y, m1_z = (line_d_front_CCS.PointAt(0.5).X, line_d_front_CCS.PointAt(0.5).Y, line_d_front_CCS.PointAt(0.5).Z)
            v1 = line_d_front_CCS.Direction
        
        v1.Unitize()
        v1_x, v1_y, v1_z = v1    
        
        m2_x, m2_y, m2_z, v2_x, v2_y, v2_z = (-1,-1,-1,-1,-1,-1)
        
        if with_random: 
            rvec = rg.Vector3d(random.uniform(-rval,rval), random.uniform(-rval,rval), random.uniform(-rval,rval))
            m0_x, m0_y, m0_z = rg.Point3d(m0_x, m0_y, m0_z) + rvec
            m1_x, m1_y, m1_z = rg.Point3d(m1_x, m1_y, m1_z) + rvec

        
        line_est_values = m0_x, m0_y, m0_z, v0_x, v0_y, v0_z, m1_x, m1_y, m1_z, v1_x, v1_y, v1_z, m2_x, m2_y, m2_z, v2_x, v2_y, v2_z
        
        #if with_random:
        #    line_est_values = [v+random.uniform(-rval,rval) for v in line_est_values]
            
        return line_est_values
    
    def get_lines_from_estimation_values_CCS(self, line_est_values):
        """ return the lines from the estimation values from the tool vision system 
        m0: midpoint of the estimated continuous wire
        v0: vector unitized of the estimated continuous wire
        m1: midpoint of the estimated discrete wire (front or back, according to the estimation type)
        v1: vector of the estimated discrete wire (front or back, according to the estimation type)
        """
        
        m0_x, m0_y, m0_z, v0_x, v0_y, v0_z, m1_x, m1_y, m1_z, v1_x, v1_y, v1_z, m2_x, m2_y, m2_z, v2_x, v2_y, v2_z  = line_est_values
        len_c = self.lines_for_estimation_WCS[0].Length/2
        len_d = self.lines_for_estimation_WCS[1].Length/2 if self.lines_for_estimation_WCS[1] else self.lines_for_estimation_WCS[2].Length/2
        
        cont_line_pt_mid = rg.Point3d(m0_x, m0_y, m0_z)
        cont_line_vec = rg.Vector3d(v0_x, v0_y, v0_z)
        cont_line_pt_from = cont_line_pt_mid - cont_line_vec * len_c
        cont_line_pt_to = cont_line_pt_mid + cont_line_vec * len_c
        cont_line_in_cam_frame = rg.Line(cont_line_pt_from, cont_line_pt_to)
        
        discrete_line_pt_mid = rg.Point3d(m1_x, m1_y, m1_z)
        discrete_line_vec = rg.Vector3d(v1_x, v1_y, v1_z)
        discrete_line_pt_from = discrete_line_pt_mid - discrete_line_vec * len_d
        discrete_line_pt_to = discrete_line_pt_mid + discrete_line_vec * len_d
        discrete_line_in_cam_frame = rg.Line(discrete_line_pt_from, discrete_line_pt_to)
        
        self.estimated_lines_CCS = [cont_line_in_cam_frame, discrete_line_in_cam_frame]
        
        return [cont_line_in_cam_frame, discrete_line_in_cam_frame]
    
    def get_lines_from_estimation_values_WCS(self, line_est_values, cam_plane_trfd_WCS):
        """ return the lines from the estimation values from the tool vision system 
        in the world coordinate system
        """
          
        estimated_lines_CCS = self.get_lines_from_estimation_values_CCS(line_est_values)
        
        T_C_W = rg.Transform.ChangeBasis(cam_plane_trfd_WCS, rg.Plane.WorldXY)
        self.estimated_lines_WCS = ghcomp.Transform(estimated_lines_CCS, T_C_W)
        
        return self.estimated_lines_WCS
    
    def get_corr_val(self, thresh_val=15):
        
        T_proj = rg.Transform.PlanarProjection(self.plane_for_inserting)
        T_in_zero = rg.Transform.PlaneToPlane(self.plane_for_inserting, rg.Plane.WorldXY)
        
        mpoint_estimated_line_WCS = self.estimated_lines_WCS[0].PointAt(0.5)
        mpoint_expected_line_WCS = self.lines_for_estimation_WCS[0].PointAt(0.5)
        
        l = rg.Line(mpoint_expected_line_WCS, mpoint_estimated_line_WCS)
        l_proj = ghcomp.Transform(l, T_proj)

        l_in_zero = ghcomp.Transform(l_proj, T_in_zero)
        vec_in_zero = l_in_zero.Direction
        
        corr_val = vec_in_zero[1] * -1
        
        if corr_val > thresh_val:
            corr_val = thresh_val
        if corr_val < -thresh_val:
            corr_val = -thresh_val
        
        self.corr_val = corr_val
        
        return corr_val, l_proj, l_in_zero
    
    def get_corr_planes(self, corr_val):
         
        self.plane_for_inserting_corr = rg.Plane(self.plane_for_inserting)
        self.plane_for_bending_corr = rg.Plane(self.plane_for_bending) if self.plane_for_bending else self.plane_wrist
        
        
        T_corr_i = rg.Transform.Translation(self.plane_for_inserting_corr.YAxis*corr_val)
        T_corr_b = rg.Transform.Translation(self.plane_for_bending_corr.YAxis*corr_val)
        self.plane_for_inserting_corr.Transform(T_corr_i)
        self.plane_for_bending_corr.Transform(T_corr_b)
        
        return self.plane_for_inserting_corr, self.plane_for_bending_corr
    
    """
    def get_lines_from_estimation_values_in_cam_frame_old(self, line_est_values, lines_for_est_global):
        
        m0_x, m0_y, m0_z, v0_x, v0_y, v0_z, m1_x, m1_y, m1_z, v1_x, v1_y, v1_z, m2_x, m2_y, m2_z, v2_x, v2_y, v2_z  = line_est_values
    
        cont_line_pt_mid = rg.Point3d(m0_x, m0_y, m0_z)
        cont_line_vec = rg.Vector3d(v0_x, v0_y, v0_z)
        cont_line_pt_from = cont_line_pt_mid - cont_line_vec * lines_for_est_global[0].Length/2
        cont_line_pt_to = cont_line_pt_mid + cont_line_vec * lines_for_est_global[0].Length/2
        cont_line_in_cam_frame = rg.Line(cont_line_pt_from, cont_line_pt_to)
        
        discrete_line_pt_mid = rg.Point3d(m1_x, m1_y, m1_z)
        discrete_line_vec = rg.Vector3d(v1_x, v1_y, v1_z)
        discrete_line_pt_from = discrete_line_pt_mid - discrete_line_vec * lines_for_est_global[1].Length/2
        discrete_line_pt_to = discrete_line_pt_mid + discrete_line_vec * lines_for_est_global[1].Length/2
        discrete_line_in_cam_frame = rg.Line(discrete_line_pt_from, discrete_line_pt_to)
        
        self.estimated_lines_in_cam_frame = [cont_line_in_cam_frame, discrete_line_in_cam_frame]
        
        return self.estimated_lines_in_cam_frame
    """

    """    
    def get_neighbor_data_for_est_global_old(self):
        ''' this method returns the lines for the line estimation with the endeffector cameras
        line 1 = line_c = line from discrete element 01 startpt to discrete element 02 startpt (= continuous element)
        line 2 = line_d1 = line of discrete element 01 (= discrete element in front)
        line 3 = line_d2 = line of discrete element 02 (= discrete element in the back)
        '''
        
        if self.neighbor_node_for_est_01 or self.neighbor_node_for_est_02:
            
            if self.neighbor_node_for_est_01 and self.neighbor_node_for_est_02:
                line_d1 = self.neighbor_node_for_est_01.line_discrete_element
                line_d2 = self.neighbor_node_for_est_02.line_discrete_element
                line_c = rg.Line(line_d1.From, line_d2.From)
            elif self.neighbor_node_for_est_01 and self.neighbor_node_for_est_02 == None:
                #print "HELLO 1"
                line_d1 = self.neighbor_node_for_est_01.line_discrete_element
                line_d2 = None #self.neighbor_node_for_est_01.line_discrete_element # 
                line_c = rg.Line(line_d1.From, self.line_discrete_element.To)
            elif self.neighbor_node_for_est_01 == None and self.neighbor_node_for_est_02:
                #print "HELLO 2"
                line_d1 = None #self.neighbor_node_for_est_02.line_discrete_element # 
                line_d2 = self.neighbor_node_for_est_02.line_discrete_element
                line_c = rg.Line(self.line_discrete_element.To, line_d2.From)

        return [line_c, line_d1, line_d2]
    """
    
    """
    def get_neighbor_data_for_est_in_cam_frame_old(self, cam_plane_trfd_global, return_as_lines = False):
        ''' this method returns the lines for the line estimation with the endeffector cameras in the camera frame
        if return_as_lines = True > return the data as lines, otherwise as a list of 3*6 (= 18) floats
        line 1 = line_c = line from discrete element 01 startpt to discrete element 02 startpt (= continuous element)
        line 2 = line_d1 = line of discrete element 01 (= discrete element in front)
        line 3 = line_d2 = line of discrete element 02 (= discrete element in the back)
        '''
        
        lines_for_est_global = self.get_neighbor_data_for_est_global()
        
        #print len(lines_for_est_global)
        #print lines_for_est_global
        
        T_from_world_to_cam = rg.Transform.ChangeBasis(rg.Plane.WorldXY, cam_plane_trfd_global)
        lines_for_est_in_cam_frame = ghcomp.Transform(lines_for_est_global, T_from_world_to_cam)
        #wire_for_estimation_in_cam_frame = 
        
        if return_as_lines:
            return lines_for_est_in_cam_frame
        
        else:
            
            if len(lines_for_est_in_cam_frame) == 3:
            
                l1, l2, l3 = lines_for_est_in_cam_frame
                l1_values = [l1.FromX, l1.FromY, l1.FromZ, l1.ToX, l1.ToY, l1.ToZ] 
                l2_values = [l2.FromX, l2.FromY, l2.FromZ, l2.ToX, l2.ToY, l2.ToZ] #if l2 else [-1 for i in range(6)]
                l3_values = [l3.FromX, l3.FromY, l3.FromZ, l3.ToX, l3.ToY, l3.ToZ] #if l3 else [-1 for i in range(6)]
                l = l1_values + l2_values + l3_values
                
                return [v/1000 for v in l]
            
            else:
                l1, l2 = lines_for_est_in_cam_frame
                l1_values = [l1.FromX, l1.FromY, l1.FromZ, l1.ToX, l1.ToY, l1.ToZ] 
                l2_values = [l2.FromX, l2.FromY, l2.FromZ, l2.ToX, l2.ToY, l2.ToZ]
                l3_values = [-1 for i in range(6)]
                
                if self.estimation_type == 1:
                    l = l1_values + l2_values + l3_values
                else:
                    l = l1_values + l3_values + l2_values
                
                return [v/1000 for v in l]
    """  
    
    """    
    def get_lines_from_estimation_values_in_cam_frame_old(self, line_est_values, lines_for_est_global):
        
        m0_x, m0_y, m0_z, v0_x, v0_y, v0_z, m1_x, m1_y, m1_z, v1_x, v1_y, v1_z, m2_x, m2_y, m2_z, v2_x, v2_y, v2_z  = line_est_values
    
        cont_line_pt_mid = rg.Point3d(m0_x, m0_y, m0_z)
        cont_line_vec = rg.Vector3d(v0_x, v0_y, v0_z)
        cont_line_pt_from = cont_line_pt_mid - cont_line_vec * lines_for_est_global[0].Length/2
        cont_line_pt_to = cont_line_pt_mid + cont_line_vec * lines_for_est_global[0].Length/2
        cont_line_in_cam_frame = rg.Line(cont_line_pt_from, cont_line_pt_to)
        
        discrete_line_pt_mid = rg.Point3d(m1_x, m1_y, m1_z)
        discrete_line_vec = rg.Vector3d(v1_x, v1_y, v1_z)
        discrete_line_pt_from = discrete_line_pt_mid - discrete_line_vec * lines_for_est_global[1].Length/2
        discrete_line_pt_to = discrete_line_pt_mid + discrete_line_vec * lines_for_est_global[1].Length/2
        discrete_line_in_cam_frame = rg.Line(discrete_line_pt_from, discrete_line_pt_to)
        
        self.estimated_lines_in_cam_frame = [cont_line_in_cam_frame, discrete_line_in_cam_frame]
        
        return self.estimated_lines_in_cam_frame
    """
    """
    def get_lines_from_estimation_values_global_old(self, line_est_values, lines_for_est_global, cam_plane_trfd_global):
          
        estimated_lines_in_cam_frame = self.get_lines_from_estimation_values_in_cam_frame(line_est_values, lines_for_est_global)
        
        T_from_cam_to_world = rg.Transform.ChangeBasis(cam_plane_trfd_global, rg.Plane.WorldXY)
        self.estimated_lines_global = ghcomp.Transform(estimated_lines_in_cam_frame, T_from_cam_to_world)
        
        return self.estimated_lines_global
    """
    
    def get_closest_point_to_other_line(self, this_line, other_line):
    
        n = rg.Vector3d.CrossProduct(this_line.Direction, other_line.Direction)
        n2 = rg.Vector3d.CrossProduct(other_line.Direction, n)
        c1 = this_line.From + (( (other_line.From - this_line.From) * n2) / (this_line.Direction * n2)) * this_line.Direction
        return c1
    
    def get_T_from_lines_exp_to_lines_est(self, lines_for_est_global, current_estimated_lines_global):
        
        #p1 = point on estimated line_c
        #p2_a = closest point on estimated line_c to estimated line_d
        #p2_b = closest point on estimated line_d to estimated line_c
        #p3 = point on estimated line_d + vector from p2_a to p2_b

        # 1. ----> get the approximated plane from the estimated lines
        line_c_exp, line_d1_exp, line_d2_exp = lines_for_est_global
        line_c_est, l_discrete_est = current_estimated_lines_global
        
        l_discrete_exp = line_d2_exp if self.estimation_type == 2 else line_d1_exp
        
        p1 = line_c_est.PointAt(0.5)
        p2_a = self.get_closest_point_to_other_line(line_c_est, l_discrete_est) # get the point on the estimated line_continuous closest to the estimated line_d2
        p2_b = self.get_closest_point_to_other_line(l_discrete_est, line_c_est) # get the point on the estimated line_d2 closest to the estimated line_continuous
        v_p2_b_to_p2_a = rg.Vector3d(p2_a-p2_b)
        p3 = l_discrete_est.PointAt(0.5) + v_p2_b_to_p2_a
        
        plane_from_est_lines = rg.Plane(p2_a, p1, p3)
        
        # 2. ----> get the approximated plane from the expected lines
        p1 = line_c_exp.PointAt(0.5)
        p2_a = self.get_closest_point_to_other_line(line_c_exp, l_discrete_exp) # get the point on the estimated line_continuous closest to the estimated line_d2
        p2_b = self.get_closest_point_to_other_line(l_discrete_exp, line_c_exp) # get the point on the estimated line_d2 closest to the estimated line_continuous
        v_p2_b_to_p2_a = rg.Vector3d(p2_a-p2_b)
        p3 = l_discrete_exp.PointAt(0.5) + v_p2_b_to_p2_a
        
        plane_from_exp_lines = rg.Plane(p2_a, p1, p3)
        
        # 3. ----> now get the transformation from the expected line plane to the estimated line plane 
        T_exp_est = rg.Transform.PlaneToPlane(plane_from_exp_lines, plane_from_est_lines)
        T_est_exp = rg.Transform.PlaneToPlane(plane_from_est_lines, plane_from_exp_lines)
        
        return plane_from_est_lines, plane_from_exp_lines, T_exp_est, T_est_exp
    
    
    def get_T_from_lines_exp_to_lines_est_2(self, lines_for_est_global, current_estimated_lines_global):
        
        #p1 = point on estimated line_c
        #p2_a = closest point on estimated line_c to estimated line_d
        #p2_b = closest point on estimated line_d to estimated line_c
        #p3 = point on estimated line_d + vector from p2_a to p2_b
        
        # 1. ----> get the approximated plane from the estimated lines
        line_c_exp, line_d1_exp, line_d2_exp = lines_for_est_global
        line_c_est, l_discrete_est = current_estimated_lines_global
        
        l_discrete_exp = line_d2_exp if self.estimation_type == 2 else line_d1_exp
        
        p1 = line_c_est.PointAt(0.5)
        p2_a = self.get_closest_point_to_other_line(line_c_est, l_discrete_est) # get the point on the estimated line_continuous closest to the estimated line_d2
        p2_b = self.get_closest_point_to_other_line(l_discrete_est, line_c_est) # get the point on the estimated line_d2 closest to the estimated line_continuous
        v_p2_b_to_p2_a = rg.Vector3d(p2_a-p2_b)
        p3 = l_discrete_est.PointAt(0.5) + v_p2_b_to_p2_a
        
        plane_from_est_lines = rg.Plane(p2_a, p1, p3)
        
        # 2. ----> get the approximated plane from the expected lines
        p1 = line_c_exp.PointAt(0.5)
        p2_a = self.get_closest_point_to_other_line(line_c_exp, l_discrete_exp) # get the point on the estimated line_continuous closest to the estimated line_d2
        p2_b = self.get_closest_point_to_other_line(l_discrete_exp, line_c_exp) # get the point on the estimated line_d2 closest to the estimated line_continuous
        v_p2_b_to_p2_a = rg.Vector3d(p2_a-p2_b)
        
        # now get the p3 approximated from the expected line
        p3 = p2_a + l_discrete_exp.Direction
        
        plane_from_exp_lines = rg.Plane(p2_a, p1, p3)
        
        # 3. ----> now get the transformation from the expected line plane to the estimated line plane 
        T_exp_est = rg.Transform.PlaneToPlane(plane_from_exp_lines, plane_from_est_lines)
        T_est_exp = rg.Transform.PlaneToPlane(plane_from_est_lines, plane_from_exp_lines)
        
        return plane_from_est_lines, plane_from_exp_lines, T_exp_est, T_est_exp
