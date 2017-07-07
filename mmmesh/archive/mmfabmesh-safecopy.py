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

Created on 21.01.2017

@author: kathrind
'''
import Rhino.Geometry as rg
import ghpythonlib.components as ghcomp
import math as m
from System.Collections.Generic import IEnumerable, List
from mmmesh import MMMesh
from node import Node

# globals
discrete_insertion_offset = 35
discrete_insertion_bending_offset = 0 #--> = formerly 8
discrete_offset_to_wire_continuous = 4.5 #6.0 #4.5
discrete_excess_len_top = 13 #0 #15
discrete_excess_len_bottom = 5 #0 #10

offset_lead_in = 5
offset_lead_out = 20
offset_to_plinth = 400

class MMFabMesh(MMMesh):

    #===========================================================================
    def __init__(self):
        MMMesh.__init__(self)
        
        # mesh nodes
        self.nodes_in_layers = [] #layer based structure [[[nodes_interior],[nodes_exterior_srf1],[nodes_exterior_srf2]],[[],[],[]],...]
        self.nodes = [] #shallow list of nodes
        
        self.collision_geo = None
    
    #===========================================================================
    def set_collision_geo(self, geo):
        self.collision_geo = geo
    
    #===========================================================================
    def generate_fab_mesh(self, row_from, row_to):
        ''' generate the mesh node topology of the intersection pts grid'''
        self.nodes_in_layers = []
        self.nodes = []
        
        self.nodes_in_layers = self.generate_nodes_in_layers(row_from, row_to)
        self.nodes = self.get_flat_list_from_2dlist([j for i in self.nodes_in_layers for j in i])
        
        self.calc_node_indices_from_nodes()
    
    def calc_neighbor_nodes_for_line_est(self):
        '''calculate the neighbor geometry for all nodes for the line estimation'''
        [self.calc_neighbor_nodes_geo_for_line_est(n) for n in self.nodes]

    #===========================================================================
    def generate_nodes_in_layers(self, row_from, row_to):
        '''calculcate the nodes in layers, starting with srf1, srf2 and each x row interiors'''
        
        nodes_in_row_interior = [self.get_fab_nodes_row_interior(row, index_row, index_layer_list=0) if row%3==0 and row>row_from+3 else [] for index_row, row in enumerate(range(max(row_from,1), row_to))]
        nodes_in_row_srf1 = [self.get_fab_nodes_row(row, index_row, "srf1", index_layer_list=1) for index_row, row in enumerate(range(max(row_from,1), row_to))]
        nodes_in_row_srf2 = [self.get_fab_nodes_row(row, index_row, "srf2", index_layer_list=2) for index_row, row in enumerate(range(max(row_from,1), row_to))]
        
        #return zip(nodes_in_row_srf1, nodes_in_row_srf2)
        return zip(nodes_in_row_interior, nodes_in_row_srf1, nodes_in_row_srf2)

    #===========================================================================
    def get_fab_nodes_row(self, row, index_row, srf = "srf1", index_layer_list=0):
        "return nodes of one row"
        
        layer_type = "even" if row%2 == 0 else "odd"
        node_type = "ext_srf1" if srf== "srf1" else "ext_srf2"
        
        nodes_in_row = []
        
        if layer_type == "even":
        
            #for col, pt in enumerate(self.get_pts_in_row_rgmesh_fab(row, srf=srf)):
            
            # 1. lead in nodes
            nodes = self.generate_nodes_exterior_lead_in(row, index_row, 0, node_type = node_type, srf = srf, layer_type = layer_type, index_layer_list = index_layer_list)
            nodes_in_row += nodes
            
            # 2. normal nodes
            for col in range(2, self.cols):
                if col==0: # lead in nodes
                    pass
                #elif col==self.cols: # lead out nodes
                #    pass 
                else:
                    if col % 2 == 0: #and col >= 2: # just take every 3rd plane
                        node = self.generate_node_exterior(row, index_row, col, node_type = node_type, srf = srf, insert_discrete_element = True, layer_type = layer_type, index_layer_list = index_layer_list)
                        nodes_in_row.append(node)
            
            # 3. lead out nodes
            nodes = self.generate_nodes_exterior_lead_out(row, index_row, node_type = node_type, srf = srf, layer_type = layer_type, index_layer_list = index_layer_list)
            nodes_in_row += nodes
        
        else: #layer_type == "odd":
            
            # 1. lead in nodes
            nodes = self.generate_nodes_exterior_lead_in(row, index_row, 0, node_type = node_type, srf = srf, layer_type = layer_type, index_layer_list = index_layer_list)
            nodes_in_row += nodes
            
            #for col, pt in enumerate(self.get_pts_in_row_rgmesh_fab(row, srf=srf)):
            for col in range(2, self.cols):
                if col==0: # lead in nodes
                    pass
                #elif col==self.cols: # lead out nodes
                #    pass 
                else:
                    if col % 2 == 1: #and col >= 2: # just take every 3rd plane
                        node = self.generate_node_exterior(row, index_row, col, node_type = node_type, srf = srf, insert_discrete_element = True, layer_type = layer_type, index_layer_list = index_layer_list)
                        nodes_in_row.append(node)
            
            # 3. lead out nodes
            nodes = self.generate_nodes_exterior_lead_out(row, index_row, node_type = node_type, srf = srf, layer_type = layer_type, index_layer_list = index_layer_list)
            nodes_in_row += nodes
        
        self.calc_lines_in_cont_dir_row(nodes_in_row) # calculate the continuous members
        self.calc_bending_angles_row(nodes_in_row) # calculate the absolute bending angles of the individual nodes
        self.calc_node_indices_col_in_row(nodes_in_row)
        self.set_collision_geo_for_nodes_in_row(nodes_in_row) # calculate the collision geo associated with every node which has a insertion
        
        return nodes_in_row
    
    #===========================================================================
    def get_fab_nodes_row_interior(self, row, index_row, index_layer_list=2):
        "return nodes of one row"
        
        
        layer_type = "even" if row%2 == 0 else "odd"
        node_type = "interior"
        
        nodes_in_row = []
        
        if layer_type == "even":
        
            #for col, pt in enumerate(self.get_pts_in_row_rgmesh_fab(row, srf=srf)):
            
            # 1. lead in nodes
            #nodes = self.generate_nodes_exterior_lead_in(row, 0, node_type = node_type, srf = srf, layer_type = layer_type, index_layer_list = index_layer_list)
            #nodes_in_row += nodes
            
            # 2. normal nodes
            for col in range(2, self.cols):
                if col==0: # lead in nodes
                    pass
                #elif col==self.cols: # lead out nodes
                #    pass 
                else:
                    if col % 2 == 0: #and col >= 2: # just take every 3rd plane
                        srf = "srf2"
                        if ((col/2)%8 == 0 or (col/2)%8 == 1 or (col/2)%8 == 2 or (col/2)%8 == 3): srf = "srf1"   
                        ide = True
                        if col>5:
                            ide = False if (col/2)%8 == 0 or (col/2)%8 == 4 else True
                        node = self.generate_node_interior(row, index_row, col, node_type = node_type, srf = srf, insert_discrete_element = ide, layer_type = layer_type, index_layer_list = index_layer_list)
                        nodes_in_row.append(node)
            
            # 3. lead out nodes
            #nodes = self.generate_nodes_exterior_lead_out(row, node_type = node_type, srf = srf, layer_type = layer_type, index_layer_list = index_layer_list)
            #nodes_in_row += nodes
        
        else: #layer_type == "odd":
            
            # 1. lead in nodes
            #nodes = self.generate_nodes_exterior_lead_in(row, 0, node_type = node_type, srf = srf, layer_type = layer_type, index_layer_list = index_layer_list)
            #nodes_in_row += nodes
            
            #for col, pt in enumerate(self.get_pts_in_row_rgmesh_fab(row, srf=srf)):
            for col in range(2, self.cols):
                if col==0: # lead in nodes
                    pass
                #elif col==self.cols: # lead out nodes
                #    pass 
                else:
                    if col % 2 == 1: #and col >= 2: # just take every 3rd plane
                        
                        srf = "srf1"
                        if ((col/2)%8 == 0 or (col/2)%8 == 1 or (col/2)%8 == 2 or (col/2)%8 == 3): srf = "srf2"   
                        ide = True
                        if col>5:
                            ide = False if (col/2)%8 == 0 or (col/2)%8 == 4 else True
                        
                        node = self.generate_node_interior(row, index_row, col, node_type = node_type, srf = srf, insert_discrete_element = ide, layer_type = layer_type, index_layer_list = index_layer_list)
                        nodes_in_row.append(node)
            
            # 3. lead out nodes
            #nodes = self.generate_nodes_exterior_lead_out(row, node_type = node_type, srf = srf, layer_type = layer_type, index_layer_list = index_layer_list)
            #nodes_in_row += nodes
        
        self.calc_lines_in_cont_dir_row(nodes_in_row) # calculate the continuous members
        self.calc_bending_angles_row(nodes_in_row) # calculate the absolute bending angles of the individual nodes
        self.calc_node_indices_col_in_row(nodes_in_row)
        self.set_collision_geo_for_nodes_in_row(nodes_in_row) # calculate the collision geo associated with every node which has a insertion
        
        return nodes_in_row
    
    def get_node_in_layer(self, row, col, index_layer_list = 0):
        ''' return the node given by row and column and side of the mesh '''
        row = min(len(self.nodes_in_layers)-1, row)
        col = min(len(self.nodes_in_layers[row][index_layer_list])-1, col)
        return self.nodes_in_layers[row][index_layer_list][col]
    
    def get_corner_pts(self, row, col, srf = "srf1"):
        pt1_row_current = self.get_pt_row_col_rgmesh_fab(row, col, srf = srf)
        pt2_row_current = self.get_pt_row_col_rgmesh_fab(row, col - 2, srf = srf)
        pt1_row_under = self.get_pt_row_col_rgmesh_fab(row - 1, col, srf = srf)
        pt2_row_under = self.get_pt_row_col_rgmesh_fab(row - 1, col - 2, srf = srf)
        return (pt1_row_current, pt2_row_current, pt1_row_under, pt2_row_under)
    
    def generate_node_exterior(self, row, index_row, col, node_type, srf, insert_discrete_element = True, layer_type = "even", index_layer_list = 0):
        ''' creating an exterior node '''
        
        node = Node(node_type, layer_type, index_row, index_layer_list) # create one instance of the class node
        
        # get the wrist_xy_plane from the cutting plane list
        wrist_plane = self.cutting_planes[row]
        
        # get the corner points of the current square
        pt_current, pt_before, pt_under_current, pt_under_before = self.get_corner_pts(row, col, srf = srf)
        
        axis = rg.Vector3d(pt_current-pt_before)
        axis.Unitize()
        
        if insert_discrete_element == True:
            
            line_discrete_element = rg.Line(pt_before, pt_under_before)
            intersect_line_cont_disc = rg.Point3d(pt_before)
            
            line_discrete_element_vec = line_discrete_element.Direction
            line_discrete_element_vec.Unitize()
            
            T_stretch_above = rg.Transform.Translation(line_discrete_element_vec * -discrete_excess_len_top)
            T_stretch_under = rg.Transform.Translation(line_discrete_element_vec * discrete_excess_len_bottom)
            
            pt_before_stretched, pt_under_before_stretched = (rg.Point3d(pt_before), rg.Point3d(pt_under_before))
            
            pt_before_stretched.Transform(T_stretch_above)
            pt_under_before_stretched.Transform(T_stretch_under)
            
            line_discrete_element = rg.Line(pt_before_stretched, pt_under_before_stretched)

            #wrist_plane_for_bending, wrist_plane_for_inserting = self.get_wrist_planes(wrist_plane, pt_before, axis)
            wrist_plane_for_inserting = self.get_wrist_plane_for_inserting(wrist_plane, pt_before, axis)
            wrist_plane_for_bending = rg.Plane(wrist_plane_for_inserting)
            wrist_plane_for_inserting_tilted = self.get_wrist_plane_for_inserting_tilted(wrist_plane_for_inserting, line_discrete_element, axis)
            
            T_offset = rg.Transform.Translation(wrist_plane_for_inserting.YAxis * -discrete_offset_to_wire_continuous)
            line_discrete_element.Transform(T_offset)
            
            
        else:
            
            line_discrete_element = None
            intersect_line_cont_disc = None
            
            wrist_plane_for_bending = rg.Plane(wrist_plane)
            wrist_plane_for_bending.Origin = pt_current
            wrist_plane_for_inserting, wrist_plane_for_inserting_tilted = (None, None)
            
        
        
        node.fill_params(wrist_plane_for_bending, axis, insert_discrete_element, wrist_plane_for_inserting, wrist_plane_for_inserting_tilted, line_discrete_element, intersect_line_cont_disc)
        
        
        #node.corner_pts = self.get_corner_pts(row, col, srf = srf)
        return node

    def generate_nodes_exterior_lead_in(self, row, index_row, col, node_type, srf, layer_type, index_layer_list):
        ''' creating the exterior node for lead in '''
        
        node_lead_in_0 = Node(node_type, layer_type, index_row, index_layer_list) # create one instance of the class node
        node_lead_in_1 = Node(node_type, layer_type, index_row, index_layer_list) # create one instance of the class node
        
        # get the wrist_xy_plane from the cutting plane list
        wrist_plane = self.cutting_planes[row]
        
        wrist_plane_for_bending_1 = rg.Plane(wrist_plane)
        wrist_plane_for_bending_1.Origin = self.get_pt_row_col_rgmesh_fab(row, col, srf = srf)
        
        node_lead_in_1.fill_params(wrist_plane_for_bending_1)  
        
        wrist_plane_for_bending_0 = rg.Plane(wrist_plane_for_bending_1)
        
        #vec_lead_in = self.get_pt_row_col_rgmesh_fab(row, 1, srf = srf) - self.get_pt_row_col_rgmesh_fab(row, 0, srf = srf)
        #vec_lead_in.Unitize()
        
        wrist_plane_for_bending_0.Translate(wrist_plane_for_bending_0.XAxis * -offset_lead_in)
        
        node_lead_in_0.fill_params(wrist_plane_for_bending_0)  
        
        #node_lead_in_1.estimate_with_eeff_cam = False
        
        return [node_lead_in_0, node_lead_in_1]

    def generate_nodes_exterior_lead_out(self, row, index_row, node_type, srf, layer_type, index_layer_list):
        ''' creating an exterior node '''
        
        node_lead_out_0 = Node(node_type, layer_type, index_row, index_layer_list) # create one instance of the class node
        node_lead_out_1 = Node(node_type, layer_type, index_row, index_layer_list) # create one instance of the class node
        
        # get the wrist_xy_plane from the cutting plane list
        wrist_plane = self.cutting_planes[row]
        
        # get the corner points of the current square
        # pt_current, pt_before, pt_under_current, pt_under_before = self.get_corner_pts(row, col, srf = srf)
        
        if layer_type == "even":
            pt_current = self.get_pt_row_col_rgmesh_fab(row, self.cols-1, srf = srf)
            pt_before = self.get_pt_row_col_rgmesh_fab(row, self.cols-2, srf = srf)
            pt_under_current = self.get_pt_row_col_rgmesh_fab(row - 1, self.cols-1, srf = srf)
            pt_under_before = self.get_pt_row_col_rgmesh_fab(row - 1, self.cols-2, srf = srf)
        else:
            vec_lead_out = self.get_pt_row_col_rgmesh_fab(row, self.cols-1, srf = srf) - self.get_pt_row_col_rgmesh_fab(row, self.cols-2, srf = srf)
            vec_lead_out.Unitize()
            
            pt_current = self.get_pt_row_col_rgmesh_fab(row, self.cols-1, srf = srf) + vec_lead_out*100
            pt_before = self.get_pt_row_col_rgmesh_fab(row, self.cols-1, srf = srf)
            pt_under_current = self.get_pt_row_col_rgmesh_fab(row - 1, self.cols-1, srf = srf) + vec_lead_out*100
            pt_under_before = self.get_pt_row_col_rgmesh_fab(row - 1, self.cols-1, srf = srf)
            
        
        axis = rg.Vector3d(pt_current-pt_before)
        axis.Unitize()
        
            
        line_discrete_element = rg.Line(pt_before, pt_under_before)
        intersect_line_cont_disc = rg.Point3d(pt_before)
        
        line_discrete_element_vec = line_discrete_element.Direction
        line_discrete_element_vec.Unitize()
        
        T_stretch_above = rg.Transform.Translation(line_discrete_element_vec * -discrete_excess_len_top)
        T_stretch_under = rg.Transform.Translation(line_discrete_element_vec * discrete_excess_len_bottom)
        
        pt_before_stretched, pt_under_before_stretched = (rg.Point3d(pt_before), rg.Point3d(pt_under_before))
        
        pt_before_stretched.Transform(T_stretch_above)
        pt_under_before_stretched.Transform(T_stretch_under)
        
        line_discrete_element = rg.Line(pt_before_stretched, pt_under_before_stretched)

        #wrist_plane_for_bending, wrist_plane_for_inserting = self.get_wrist_planes(wrist_plane, pt_before, axis)
        wrist_plane_for_inserting = self.get_wrist_plane_for_inserting(wrist_plane, pt_before, axis)
        wrist_plane_for_bending = rg.Plane(wrist_plane_for_inserting)
        wrist_plane_for_inserting_tilted = self.get_wrist_plane_for_inserting_tilted(wrist_plane_for_inserting, line_discrete_element, axis)
        
        T_offset = rg.Transform.Translation(wrist_plane_for_inserting.YAxis * -discrete_offset_to_wire_continuous)
        line_discrete_element.Transform(T_offset)
        
        
        node_lead_out_0.fill_params(wrist_plane_for_bending, axis, True, wrist_plane_for_inserting, wrist_plane_for_inserting_tilted, line_discrete_element, intersect_line_cont_disc)
        
        
        #node_lead_out_0.corner_pts = [pt_current, pt_before, pt_under_current, pt_under_before]
        
        wrist_plane_for_bending_1 = rg.Plane(wrist_plane_for_bending)
        wrist_plane_for_bending_1.Translate(wrist_plane_for_bending_1.XAxis * offset_lead_out)
        
        node_lead_out_1.fill_params(wrist_plane_for_bending_1)  
        
        
        return [node_lead_out_0, node_lead_out_1]
    
    def get_corner_pts_interior(self, row, col, srf = "srf1"):
        ''' get corner points for interior layer '''
        
        pt1_row_current = self.get_pt_row_col_rgmesh_fab(row, col, srf = srf)
        pt2_row_current = self.get_pt_row_col_rgmesh_fab(row, col - 2, srf = srf)
        pt1_row_under = self.get_pt_row_col_rgmesh_fab(row - 1, col, srf = srf)
        pt2_row_under = self.get_pt_row_col_rgmesh_fab(row - 1, col - 2, srf = srf)
        
        vec_h = pt1_row_current - pt2_row_current
        #vec_h.Unitize()
        
        vec_v = pt2_row_current - pt2_row_under
        #vec_v.Unitize()
        
        #pt2_row_current = pt2_row_current + 20 * vec_h - 20 * vec_v
        pt1_row_current = pt1_row_current - vec_v/3*1
        pt2_row_current = pt2_row_current + vec_h/4*1 - vec_v/3*1
        pt2_row_under = pt2_row_under + vec_h/4*1
        
        return (pt1_row_current, pt2_row_current, pt1_row_under, pt2_row_under)
    
    def generate_node_interior(self, row, index_row, col, node_type, srf, insert_discrete_element = True, layer_type = "even", index_layer_list = 0):
        ''' creating an exterior node '''
        
        node = Node(node_type, layer_type, index_row, index_layer_list) # create one instance of the class node
        
        # get the wrist_xy_plane from the cutting plane list
        wrist_plane = self.cutting_planes[row]
        
        # get the corner points of the current square
        pt_current, pt_before, pt_under_current, pt_under_before = self.get_corner_pts_interior(row, col, srf = srf)
        
        axis = rg.Vector3d(pt_current-pt_before)
        axis.Unitize()
        
        if insert_discrete_element == True:
            
            line_discrete_element = rg.Line(pt_before, pt_under_before)
            intersect_line_cont_disc = rg.Point3d(pt_before)
            
            line_discrete_element_vec = line_discrete_element.Direction
            line_discrete_element_vec.Unitize()
            
            T_stretch_above = rg.Transform.Translation(line_discrete_element_vec * -discrete_excess_len_top)
            T_stretch_under = rg.Transform.Translation(line_discrete_element_vec * discrete_excess_len_bottom)
            
            pt_before_stretched, pt_under_before_stretched = (rg.Point3d(pt_before), rg.Point3d(pt_under_before))
            
            pt_before_stretched.Transform(T_stretch_above)
            pt_under_before_stretched.Transform(T_stretch_under)
            
            line_discrete_element = rg.Line(pt_before_stretched, pt_under_before_stretched)

            #wrist_plane_for_bending, wrist_plane_for_inserting = self.get_wrist_planes(wrist_plane, pt_before, axis)
            wrist_plane_for_inserting = self.get_wrist_plane_for_inserting(wrist_plane, pt_before, axis)
            wrist_plane_for_bending = rg.Plane(wrist_plane_for_inserting)
            wrist_plane_for_inserting_tilted = self.get_wrist_plane_for_inserting_tilted(wrist_plane_for_inserting, line_discrete_element, axis)
            
            T_offset = rg.Transform.Translation(wrist_plane_for_inserting.YAxis * -discrete_offset_to_wire_continuous)
            line_discrete_element.Transform(T_offset)
            
            
        else:
            
            line_discrete_element = None
            intersect_line_cont_disc = None
            
            wrist_plane_for_bending = rg.Plane(wrist_plane)
            wrist_plane_for_bending.Origin = pt_current
            wrist_plane_for_inserting, wrist_plane_for_inserting_tilted = (None, None)
            
        
        
        node.fill_params(wrist_plane_for_bending, axis, insert_discrete_element, wrist_plane_for_inserting, wrist_plane_for_inserting_tilted, line_discrete_element, intersect_line_cont_disc)
        
        
        node.corner_pts = self.get_corner_pts(row, col, srf = srf)
        return node
    
    def generate_nodes_exterior_lead_out_old(self, row, col, node_type, srf, insert_discrete_element = True, layer_type = "even"):
        ''' creating the exterior node for lead in '''
        
        lead_out_nodes = []

        if layer_type == "even":
            
            node_lead_out_0 = self.generate_node_exterior(row, self.u, node_type, srf, insert_discrete_element = True, layer_type = layer_type)
            lead_out_nodes.append(node_lead_out_0)
        
            node_lead_out_1 = Node(node_type, layer_type, row) # create one instance of the class node
            
            # get the wrist_xy_plane from the cutting plane list
            wrist_plane = self.cutting_planes[row]
            
            # get the corner points of the current square of planes
            pt_before = self.get_pt_row_col_rgmesh_fab(row, self.u, srf = srf)
            pt_under_before = self.get_pt_in_row(row-2, self.u, srf=srf)
            
            pt_current = pt_before + self.get_plane_from_srf(pt_before, srf).YAxis * 100
            pt_under_current = pt_under_before + self.get_plane_from_srf(pt_under_before, srf).YAxis * 100
            
            '''
            pt_current = pt_before + wrist_plane.YAxis * 100
            pt_under_current = pt_under_before + wrist_plane.YAxis * 100'''
            
            axis = rg.Vector3d(pt_current-pt_before)
            axis.Unitize()

            line_discrete_element = rg.Line(pt_before, pt_under_before)
            
            wrist_plane_for_bending, wrist_plane_for_inserting = self.get_wrist_planes(wrist_plane, pt_before, axis)
            wrist_plane_for_inserting_tilted = self.get_wrist_plane_for_inserting_tilted(wrist_plane_for_inserting, line_discrete_element, axis)   
            
            node_lead_out_1.fill_params(wrist_plane_for_bending, axis, insert_discrete_element, wrist_plane_for_inserting, wrist_plane_for_inserting_tilted, line_discrete_element)  
           
            node_lead_out_1.line_for_estimation = rg.Line(pt_under_before, pt_under_current)
            
            lead_out_nodes.append(node_lead_out_1)
        
        else: # layer type == "odd"
            print "odd layer"
            
            node_lead_out_0 = self.generate_node_exterior(row, col, node_type, srf, insert_discrete_element = True, layer_type = layer_type)
            lead_out_nodes.append(node_lead_out_0)
            
            node_lead_out_1 = Node(node_type, layer_type, row) # create one instance of the class node
            
            # get the wrist_xy_plane from the cutting plane list
            wrist_plane = self.cutting_planes[row]
            
            # get the corner points of the current square of planes
            pt_before = self.get_pt_in_row(row, self.u-2, srf=srf)
            pt_under_before = self.get_pt_in_row(row-2, self.u-2, srf=srf)
            pt_current = self.get_pt_in_row(row, self.u, srf=srf)
            pt_under_current = self.get_pt_in_row(row-2, self.u, srf=srf)

            axis = rg.Vector3d(pt_current-pt_before)
            axis.Unitize()

            line_discrete_element = rg.Line(pt_before, pt_under_before)
            
            wrist_plane_for_bending, wrist_plane_for_inserting = self.get_wrist_planes(wrist_plane, pt_before, axis)
            wrist_plane_for_inserting_tilted = self.get_wrist_plane_for_inserting_tilted(wrist_plane_for_inserting, line_discrete_element, axis)   
            
            node_lead_out_1.fill_params(wrist_plane_for_bending, axis, insert_discrete_element, wrist_plane_for_inserting, wrist_plane_for_inserting_tilted, line_discrete_element)  
           
            node_lead_out_1.line_for_estimation = rg.Line(pt_under_before, pt_under_current)
            
            lead_out_nodes.append(node_lead_out_1)
        
        
        node_lead_out_2 = Node(node_type, layer_type, row)
        
        wrist_plane_for_bending = rg.Plane(node_lead_out_1.wrist_plane_for_bending)
        T = rg.Transform.Translation(node_lead_out_1.axis * self.offset_lead_out)
        wrist_plane_for_bending.Transform(T)
        
        node_lead_out_2.fill_params(wrist_plane_for_bending)  
        lead_out_nodes.append(node_lead_out_2)
        
        return lead_out_nodes
    
    def get_wrist_planes_old(self, wrist_plane, pt_before, axis):
        ''' the wrist planes have an offset to the inserted vertical, given the vertical insertion offset and the additional offset for bending'''

        wrist_plane_for_bending, wrist_plane_for_inserting = (rg.Plane(wrist_plane), rg.Plane(wrist_plane))
        wrist_plane_for_bending.Origin = rg.Point3d.Add(pt_before, axis * (discrete_insertion_offset + discrete_insertion_bending_offset))
        wrist_plane_for_inserting.Origin = rg.Point3d.Add(pt_before, axis * discrete_insertion_offset)
        return wrist_plane_for_bending, wrist_plane_for_inserting
    
    def get_wrist_plane_for_inserting(self, wrist_plane, pt_before, axis):
        ''' the wrist planes have an offset to the inserted vertical, given the vertical insertion offset and the additional offset for bending'''

        wrist_plane_for_inserting = rg.Plane(wrist_plane)
        wrist_plane_for_inserting.Origin = rg.Point3d.Add(pt_before, axis * discrete_insertion_offset)
        
        return wrist_plane_for_inserting
    
    def get_wrist_plane_for_inserting_tilted(self, wrist_plane_for_inserting, line_discrete_element, axis):
        '''return xy plane tilted according to the angle of the vertical wire (= rg.Line)'''
        
        wrist_plane_for_inserting_tilted = rg.Plane(wrist_plane_for_inserting)
        angle = self.get_angle(rg.Vector3d(line_discrete_element.From - line_discrete_element.To), wrist_plane_for_inserting.ZAxis, axis)
        wrist_plane_for_inserting_tilted.Rotate(angle, axis,  wrist_plane_for_inserting_tilted.Origin)
        
        return wrist_plane_for_inserting_tilted
    
    def get_angle(self,n1,n2,axis): 
        ''' get the angle for the tilted plane for inserting the discrete rebar element'''
        
        #________________check for the rotation direction_______________________
        #angle           = rs.VectorAngle (n1,n2) # rotation angle
        angle           = rg.Vector3d.VectorAngle(rg.Vector3d(n1), rg.Vector3d(n2))
         
        #print "angle: ", angle
        test_vec_clw = rg.Vector3d(n1)
        test_vec_clw.Rotate(angle, axis)
        
        sub_vec_clw = rg.Vector3d.Subtract(test_vec_clw,n2)
        length_clw = sub_vec_clw.Length
        test_vec_aclw = rg.Vector3d(n1)
        test_vec_aclw.Rotate(angle*-1, axis)       
        sub_vec_aclw = rg.Vector3d.Subtract(test_vec_aclw, n2)
        length_aclw = sub_vec_aclw.Length
        
        if angle == 0 or length_clw < length_aclw:
            rot_angle = angle * -1
        else:
            rot_angle = angle 
   
        return rot_angle
    
    def calc_neighbor_nodes_geo_for_line_est(self, node):
        ''' this method returns the lines for the line estimation with the endeffector cameras
        line 1 = line_c = line from discrete element 01 startpt to discrete element 02 startpt (= continuous element)
        line 2 = line_d1 = line of discrete element 01 (= discrete element in front)
        line 3 = line_d2 = line of discrete element 02 (= discrete element in the back)
        '''
        
        # estimation_type: 1: front discrete wire, 2: back discrete wire, 3: match both wires
        
        neighbor_nodes = self.get_neighbor_nodes_from_node_for_line_est(node)
        if len(neighbor_nodes):
            node.estimate_with_eeff_cam = True
            node.neighbor_node_for_est_01, node.neighbor_node_for_est_02 = neighbor_nodes
            line_d1, line_d2 = [n.line_discrete_element for n in neighbor_nodes]
            
            if line_d1 and line_d2:
                #print "line_d1 and line_d2"
                node.estimation_type = 2
                p1, p2 = [n.intersect_line_cont_disc for n in neighbor_nodes]
                line_c = rg.Line(p1, p2)
                print line_c
            
            if line_d1 and line_d2 == None:
                #print "line_d1 and line_d2 == None"
                node.estimation_type = 1
                p1 = neighbor_nodes[0].intersect_line_cont_disc
                p2 = neighbor_nodes[1].wrist_plane_for_bending.Origin
                line_c = rg.Line(p1, p2)
                print line_c
            
            if line_d1 == None and line_d2:
                #print "line_d1 == None and line_d2"
                node.estimation_type = 2
                p2 = neighbor_nodes[1].intersect_line_cont_disc
                p1 = neighbor_nodes[0].wrist_plane_for_bending.Origin
                line_c = rg.Line(p1, p2)
                #print line_c
                
            if line_d1 == None and line_d2 == None:
                #print "line_d1 == None and line_d2 == None"
                node.estimation_type = 3
                p2 = neighbor_nodes[1].wrist_plane_for_bending.Origin
                p1 = neighbor_nodes[0].wrist_plane_for_bending.Origin
                line_c = rg.Line(p1, p2)
            
            
            node.estimated_lines_global = [line_c, line_d1, line_d2]


    def get_neighbor_nodes_from_node_for_line_est(self, node):
        #node.index_row
        #node.index_col
        #node.index
        #node.index_layer_list
        
        if node.insert_discrete_element and node.node_type != "interior":
            row_prev = node.index_row-1
            
            if node.layer_type == "odd":  
                neighbor_node_for_est_01 = self.get_node_in_layer(row_prev, node.index_col+1, index_layer_list = node.index_layer_list)
                neighbor_node_for_est_02 = self.get_node_in_layer(row_prev, node.index_col, index_layer_list = node.index_layer_list)
            else:
                neighbor_node_for_est_01 = self.get_node_in_layer(row_prev, node.index_col, index_layer_list = node.index_layer_list)
                neighbor_node_for_est_02 = self.get_node_in_layer(row_prev, node.index_col-1, index_layer_list = node.index_layer_list)
            
            return (neighbor_node_for_est_01, neighbor_node_for_est_02)
        else:
            return []
        
    def calc_neighbor_nodes_from_node_for_line_est(self, node):
        neighbor_nodes = self.get_neighbor_nodes_from_node_for_line_est(node)
        if len(neighbor_nodes): node.neighbor_node_for_est_01, node.neighbor_node_for_est_02 = neighbor_nodes
    
    def calc_neighbor_nodes_for_line_estimation(self):
        for layer in self.nodes_in_layers[1:]:
            for node_list in layer:
                for node in node_list[1:len(node_list)-1]:
                    self.calc_neighbor_nodes_from_node_for_line_est(node)
                    
    def calc_neighbor_nodes_for_line_estimation_old(self):
        for i, layer in enumerate(self.nodes_in_layers[1:]):
            i = i+1 # because we start only from the second layer
            for j, node_list in enumerate(layer):
                for k, node in enumerate(node_list):
                    if k > 0 and k < len(node_list)-1:
                        node_current = self.nodes_in_layers[i][j][k]
                        if node_current.layer_type == "odd":
                            #print node_current.layer_type
                            node_current.neighbor_node_for_est_01 = self.nodes_in_layers[i-1][j][k+1] if self.nodes_in_layers[i-1][j][k+1].insert_discrete_element else None
                            node_current.neighbor_node_for_est_02 = self.nodes_in_layers[i-1][j][k] if self.nodes_in_layers[i-1][j][k].insert_discrete_element else None
                        else: #node.layer_type == "even"
                            #print node_current.layer_type 
                            if k < len(self.nodes_in_layers[i-1][j]):
                                node_current.neighbor_node_for_est_01 = self.nodes_in_layers[i-1][j][k] if self.nodes_in_layers[i-1][j][k].insert_discrete_element else None
                                node_current.neighbor_node_for_est_02 = self.nodes_in_layers[i-1][j][k-1] if self.nodes_in_layers[i-1][j][k-1].insert_discrete_element else None
                            else:
                                node_current.neighbor_node_for_est_01 = None
                                node_current.neighbor_node_for_est_02 = self.nodes_in_layers[i-1][j][k-1]
                        
                        # now define which type of estimation is necessary, depending on which discrete element exists
                        if node_current.neighbor_node_for_est_01 and node_current.neighbor_node_for_est_02 == None:
                            node_current.estimation_type = 1
                            node_current.estimate_with_eeff_cam = True
                        elif node_current.neighbor_node_for_est_02 and node_current.neighbor_node_for_est_01 == None:
                            node_current.estimation_type = 2
                            node_current.estimate_with_eeff_cam = True
                        elif node_current.neighbor_node_for_est_01 and node_current.neighbor_node_for_est_02:
                            node_current.estimation_type = 2
                            node_current.estimate_with_eeff_cam = True
                        else:
                            node_current.estimate_with_eeff_cam = False
    

    
    def calc_node_indices_from_nodes(self):
        '''calculates the index of nodes from the falt list of nodes'''
        [node.set_index(i) for i, node in enumerate(self.nodes)]
        
    def calc_node_indices_col_in_row(self, nodes_in_row):
        '''calculates the index of nodes from the falt list of nodes'''
        [node.set_index_col(i) for i, node in enumerate(nodes_in_row)]

    def calc_lines_in_cont_dir_row(self, nodes_in_row):
        '''this methods takes a row of nodes and calculates the line members in contiuous direction '''
        
        # first node
        if not nodes_in_row[0].node_type == "interior":
            node_current = nodes_in_row[0]
            node_current.line_continuous_element = rg.Line(node_current.wrist_plane_for_bending.Origin, node_current.wrist_plane_for_bending.Origin - node_current.wrist_plane_for_bending.XAxis * offset_to_plinth)
            
        for i in range(1, len(nodes_in_row)):
            node_current = nodes_in_row[i]
            node_before = nodes_in_row[i-1]
            node_current.line_continuous_element = rg.Line(node_current.wrist_plane_for_bending.Origin, node_before.wrist_plane_for_bending.Origin)
                
    def calc_bending_angles_row(self, nodes_in_row):
        '''this methods takes a row of nodes and calculates the bending angles for the MM Tool. 
        The values are absolute in relation to the X axis of the current wrist plane of each layer '''             

        for i in range(len(nodes_in_row)-1):
            
            if nodes_in_row[i+1].line_continuous_element:
                vec = rg.Vector3d(nodes_in_row[i+1].line_continuous_element.From - nodes_in_row[i+1].line_continuous_element.To)
                angle = rg.Vector3d.VectorAngle(nodes_in_row[i].wrist_plane_for_bending.XAxis, vec, nodes_in_row[i].wrist_plane_for_bending)
                
                try: angle = m.degrees(angle) # needs to be done because of a math overflow error
                except: angle = 0
                
                if angle > 180:
                    angle = (360-angle)*-1

                nodes_in_row[i].absolute_bending_angle = angle
    
    def set_collision_geo_for_nodes_in_row(self, nodes_in_row):
        if self.collision_geo:
            for i, node in enumerate(nodes_in_row):
                if node.insert_discrete_element:
                    T1 = rg.Transform.PlaneToPlane(rg.Plane.WorldXY, node.wrist_plane_for_inserting_tilted)
                    bending_angle = nodes_in_row[i-1].absolute_bending_angle  if i>0 else node.absolute_bending_angle
                    #T2 = rg.Transform.Rotation(m.radians(bending_angle), node.wrist_plane_for_inserting_tilted.ZAxis, node.wrist_plane_for_inserting_tilted.Origin)
                    T2 = rg.Transform.Rotation(m.radians(bending_angle), rg.Plane.WorldXY.ZAxis, rg.Plane.WorldXY.Origin)
    
                    coll_geo_transformed = ghcomp.Transform(self.collision_geo, T1*T2)
                    node.collision_geo = coll_geo_transformed
    
    def get_collcheck_geo(self, node_idx, collcheck_num):
        mesh_nodes_collcheck = self.nodes[max(node_idx - collcheck_num, 0):node_idx]
        geo_c_collcheck = [node.line_continuous_element for node in mesh_nodes_collcheck if node.line_continuous_element]
        geo_d_collcheck = [node.line_discrete_element for node in mesh_nodes_collcheck if node.line_discrete_element]
        return geo_c_collcheck+geo_d_collcheck
        