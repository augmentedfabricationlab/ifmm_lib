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
discrete_insertion_offset = 34
discrete_insertion_bending_offset = 8

class MMFabMesh(MMMesh):
    
    def __init__(self):
        MMMesh.__init__(self)
    
    def generate_fab_mesh(self, row_from, row_to):
        ''' generate the mesh node topology of the intersection pts grid'''
        self.generate_node_grid(row_from, row_to)
        self.calc_lines_in_cont_dir_and_bending_angles_abs() # calculate the absolute bending angles of the individual nodes
        self.calc_node_indices() # calculate the node indices
        #self.calc_neighbor_nodes_for_line_estimation() # calculate the neighbor nodes for the line estimation
    
    def generate_node_grid(self, row_from, row_to):
        ''' now the individual nodes are generated from the point grid according to certain conditions
        
        the sequence (from bottom to top) always consists of: exterior_srf1, exterior_srf2, interior
        
        nodes can be of:
        - node_type: "exterior_srf1" / "exterior_srf2" / "interior"
        - special nodes are created: for the lead in / lead out sequence
        '''
        
        print "generating fabrication sequence"

        # mesh nodes
        self.nodes_in_layers = [] #layer based structure [[[nodes_interior],[nodes_exterior],[nodes_exterior]],[[.],[.],[.]],...]
        self.nodes = [] #shallow list of nodes 

        # now loop through the point grid
        for row in range(self.rows):

            if row > 0 and row > row_from and row < row_to: # only start with the second layer
                
                row_srf1_nodes = []
                row_srf2_nodes = []
                row_interior_nodes = []
                current_layer = [] # into the current layer: even: [interior, exterior_srf2, exterior_srf1], odd: [interior, exterior_srf1, exterior_srf2]
                
                # even layers
                if row % 2 == 0:
                   
                    # srf1
                    for col, pt in enumerate(self.get_pts_in_row_rgmesh_fab(row, srf="srf1")): # start with srf1
                        if col % 2 == 1: # just take every 5th plane
                            nodes_srf1 = self.generate_nodes(row, col, "srf1", "exterior_srf1", "even")
                            row_srf1_nodes += nodes_srf1
                            self.nodes += nodes_srf1
                    
                    # srf2
                    for col, pt in enumerate(self.get_pts_in_row_rgmesh_fab(row, srf="srf2")):
                        if col % 2 == 1: # just take every 5th plane
                            nodes_srf2 = self.generate_nodes(row, col, "srf2", "exterior_srf2", "even")
                            row_srf2_nodes += nodes_srf2
                            self.nodes += nodes_srf2       

                    # store into the rows
                    if len(row_interior_nodes) > 0: current_layer.append(row_interior_nodes)                       
                    if len(row_srf1_nodes) > 0: current_layer.append(row_srf1_nodes)
                    if len(row_srf2_nodes) > 0: current_layer.append(row_srf2_nodes)
                
                # odd layers
                if row % 2 == 1:
                            
                    # srf1
                    for col, pt in enumerate(self.get_pts_in_row_rgmesh_fab(row, srf="srf1")): # start with srf1
                        if col % 2 == 0: # just take every 3rd plane
                            nodes_srf1 = self.generate_nodes(row, col, "srf1", "exterior_srf1", "odd")
                            row_srf1_nodes += nodes_srf1
                            self.nodes += nodes_srf1
                    
                    # srf2
                    for col, pt in enumerate(self.get_pts_in_row_rgmesh_fab(row, srf="srf2")): # start with srf1
                        if col % 2 == 0: # just take every 3rd plane
                            nodes_srf2 = self.generate_nodes(row, col, "srf2", "exterior_srf2", "odd")
                            row_srf2_nodes += nodes_srf2
                            self.nodes += nodes_srf2
                    
                    # store into the rows
                    if len(row_interior_nodes) > 0: current_layer.append(row_interior_nodes)
                    if len(row_srf1_nodes) > 0: current_layer.append(row_srf1_nodes)
                    if len(row_srf2_nodes) > 0: current_layer.append(row_srf2_nodes)
                    
                if len(current_layer) > 0: self.nodes_in_layers.append(current_layer)
    
             
    def generate_nodes(self, row, col, srf, node_type, layer_type):
        ''' generating a node according to its node_type (exterior/interior) and layer_type (even/odd) '''
        
        nodes = []
        
        """
        if col < 3: # lead in nodes
            nodes.append(self.generate_node_exterior_lead_in(row, col, node_type, srf, insert_discrete_element = False, layer_type = layer_type))
        elif col < self.u-3: # regular node
            nodes.append(self.generate_node_exterior(row, col, node_type, srf, insert_discrete_element = True, layer_type = layer_type))
        else: # lead out nodes
            nodes += self.generate_nodes_exterior_lead_out(row, col, node_type, srf, insert_discrete_element = True, layer_type = layer_type)
        """
        if col >= 2:
            nodes.append(self.generate_node_exterior(row, col, node_type, srf, insert_discrete_element = True, layer_type = layer_type))
            
        if node_type == "interior":
            pass
                    
        return nodes
    
    def get_pt_in_row_rgmesh_fab(self, row, col, srf):
        '''return the point of the point grid given column and row and surface 1 or surface 2'''    
        #return self.intersection_pts_in_layers_srf1[row][col] if srf == 1 else self.intersection_pts_in_layers_srf2[row][col]
        return self.get_pt_row_col_rgmesh_fab(row, col, srf = srf)
    
    def get_node_in_row(self, row, col, srf = "srf1"):
        ''' return the node given by row and column and side of the mesh '''
        srf_idx = 0 if srf == "srf1" else 1
        list_length = len(self.nodes_in_layers[row][srf_idx])
        col = min(list_length-1, col)
        return self.nodes_in_layers[row][srf_idx][col]
    
    def get_plane_from_srf(self, pt, srf):
        '''get the srf plane closest tot the input point'''
        
        s = self.srf1_rebuilt if srf == 1 else self.srf2_rebuilt
        
        srf_param = s.ClosestPoint(pt)
        srf_plane = s.FrameAt(srf_param[1], srf_param[2])[1]
        
        return srf_plane
    
    def generate_node_exterior(self, row, col, node_type, srf, insert_discrete_element = True, layer_type = "even"):
        ''' creating an exterior node '''
        
        node = Node(node_type, layer_type, row) # create one instance of the class node
        
        # get the wrist_xy_plane from the cutting plane list
        wrist_plane = self.cutting_planes[row]
        
        # get the corner points of the current square of planes
        pt_current = self.get_pt_in_row_rgmesh_fab(row, col, srf=srf)
        pt_before = self.get_pt_in_row_rgmesh_fab(row, col-2, srf=srf)
        pt_under_current = self.get_pt_in_row_rgmesh_fab(row-1, col, srf=srf)
        pt_under_before = self.get_pt_in_row_rgmesh_fab(row-1, col-2, srf=srf)
        
        axis = rg.Vector3d(pt_current-pt_before)
        axis.Unitize()
        
        if insert_discrete_element == True:
            
            line_discrete_element = rg.Line(pt_before, pt_under_before)
            
            wrist_plane_for_bending, wrist_plane_for_inserting = self.get_wrist_planes(wrist_plane, pt_before, axis)
            wrist_plane_for_inserting_tilted = self.get_wrist_plane_for_inserting_tilted(wrist_plane_for_inserting, line_discrete_element, axis)
            
            T_offset = rg.Transform.Translation(wrist_plane_for_bending.YAxis * -5)
            line_discrete_element.Transform(T_offset)
            
            scale_plane = rg.Plane(line_discrete_element.PointAt(0.5), wrist_plane_for_inserting_tilted.XAxis, wrist_plane_for_inserting_tilted.YAxis)
            T_scale = rg.Transform.Scale(scale_plane, 1, 1, 1.5)
            line_discrete_element.Transform(T_scale)
            
        else:
            
            line_discrete_element = None
            
            wrist_plane_for_bending = rg.Plane(wrist_plane)
            wrist_plane_for_bending.Origin = pt_current
            wrist_plane_for_inserting, wrist_plane_for_inserting_tilted = (None, None)
            
        
        node.fill_params(wrist_plane_for_bending, axis, insert_discrete_element, wrist_plane_for_inserting, wrist_plane_for_inserting_tilted, line_discrete_element)  
        
        return node

    def generate_node_exterior_lead_in(self, row, col, node_type, srf, insert_discrete_element = False, layer_type = "even"):
        ''' creating the exterior node for lead in '''
        
        # get the wrist_xy_plane from the cutting plane list
        wrist_plane = self.cutting_planes[row]

        node_lead_in_0 = Node(node_type, layer_type, row) # create one instance of the class node
        wrist_plane_for_bending = rg.Plane(wrist_plane)
        wrist_plane_for_bending.Origin = self.get_pt_in_row(row, 0, srf=srf)
        node_lead_in_0.fill_params(wrist_plane_for_bending)  
        
        node_lead_in_0.estimate_with_eeff_cam = False
        return node_lead_in_0
    
    def generate_nodes_exterior_lead_out(self, row, col, node_type, srf, insert_discrete_element = True, layer_type = "even"):
        ''' creating the exterior node for lead in '''
        
        lead_out_nodes = []

        if layer_type == "even":
            
            node_lead_out_0 = self.generate_node_exterior(row, self.u, node_type, srf, insert_discrete_element = True, layer_type = layer_type)
            lead_out_nodes.append(node_lead_out_0)
        
            node_lead_out_1 = Node(node_type, layer_type, row) # create one instance of the class node
            
            # get the wrist_xy_plane from the cutting plane list
            wrist_plane = self.cutting_planes[row]
            
            # get the corner points of the current square of planes
            pt_before = self.get_pt_in_row(row, self.u, srf=srf)
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
    
    def get_wrist_planes(self, wrist_plane, pt_before, axis):
        ''' the wrist planes have an offset to the inserted vertical, given the vertical insertion offset and the additional offset for bending'''

        wrist_plane_for_bending, wrist_plane_for_inserting = (rg.Plane(wrist_plane), rg.Plane(wrist_plane))
        wrist_plane_for_bending.Origin = rg.Point3d.Add(pt_before, axis * (discrete_insertion_offset + discrete_insertion_bending_offset))
        wrist_plane_for_inserting.Origin = rg.Point3d.Add(pt_before, axis * discrete_insertion_offset)
        return wrist_plane_for_bending, wrist_plane_for_inserting
    
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
    
    def calc_lines_in_cont_dir_and_bending_angles_abs(self):
        '''this methods takes the existing nodes and calculates the lines incontiuous direction and bending angles for the MM Tool. 
        The values are absolute in relation to the X axis of the current wrist plane of each layer '''
        
        print "calculating absolute bending angles"
        
        for layer in self.nodes_in_layers:
            for row in layer:
                for i in range(1, len(row)):
                    node_current = row[i]
                    node_before = row[i-1]
                    node_current.line_continuous_element = rg.Line(node_current.wrist_plane_for_bending.Origin, node_before.wrist_plane_for_bending.Origin)
                    
        for layer in self.nodes_in_layers:
            for row in layer:
                for i in range(len(row)-1):
                    
                    if row[i+1].line_continuous_element:
                        vec = rg.Vector3d(row[i+1].line_continuous_element.From - row[i+1].line_continuous_element.To)
                        angle = rg.Vector3d.VectorAngle(row[i].wrist_plane_for_bending.XAxis, vec, row[i].wrist_plane_for_bending)
                        
                        try: angle = m.degrees(angle) # needs to be done because of a math overflow error
                        except: angle = 0
                        
                        if angle > 180:
                            angle = (360-angle)*-1

                        row[i].absolute_bending_angle = angle
                        

    def calc_node_indices(self):
        
        print "calculating node indices"
        self.nodes = []
        index_counter = 0
        
        for l, layer in enumerate(self.nodes_in_layers):
            node_list_counter = 0 
            for j, node_list in enumerate(layer):
                 
                for i, node in enumerate(node_list):
                    
                    node.index = index_counter # index running
                    index_counter += 1
                    
                    node.index_row = l 
                    node.index_col = i
                    node.index_layer_list = node_list_counter
                    
                    
                    if i >= 1 and i < (len(node_list)-1):
                        node.is_for_bending = True

                node_list_counter += 1
                
                """
                # only put the nodes into the nodes list from on the second layer, first layer is built already
                if l<1:
                    #print l
                    #print "groundlayer"
                    self.nodes_groundlayer += [node for node in node_list]
                else:
                    #print "regular layers"
                    self.nodes += [node for node in node_list]"""
                
                self.nodes += [node for node in node_list]
                    
    def calc_neighbor_nodes_for_line_estimation(self):
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
    
    
    def generate_fab_mesh_new(self, row_from, row_to):
        ''' generate the mesh node topology of the intersection pts grid'''
        self.nodes_in_layers = self.generate_nodes_in_rows(row_from, row_to)
        #self.nodes = self.get_flat_list_from_2dlist([j for i in self.nodes_in_layers for j in i])
        self.calc_node_indices_from_nodes_in_layers()
    
    def calc_node_indices_from_nodes_in_layers(self):
        
        print "calculating node indices"
        self.nodes = []
        index_counter = 0
        
        for l, layer in enumerate(self.nodes_in_layers):
            node_list_counter = 0 
            for j, node_list in enumerate(layer):
                 
                for i, node in enumerate(node_list):
                    
                    node.index = index_counter # index running
                    index_counter += 1
                    
                    node.index_row = l 
                    node.index_col = i
                    node.index_layer_list = node_list_counter
                    
                    
                    if i >= 1 and i < (len(node_list)-1):
                        node.is_for_bending = True

                node_list_counter += 1
                self.nodes += [node for node in node_list]   
        
        
    def generate_nodes_in_rows(self, row_from, row_to):
        
        nodes_in_row_srf1 = [self.get_fab_nodes_row(row, "srf1") for row in range(max(row_from,1), row_to)]
        nodes_in_row_srf2 = [self.get_fab_nodes_row(row, "srf2") for row in range(max(row_from,1), row_to)]
        
        return zip(nodes_in_row_srf1, nodes_in_row_srf2)
    
    #===================================================
    # maybe as an extension --> only get fab nodes for one row
    #===================================================
    def get_fab_nodes_row(self, row, srf = "srf1"):
        "return nodes of one row"
        
        layer_type = "even" if row%2 == 0 else "odd"
        node_type = "ext_srf1" if srf== "srf1" else "ext_srf2"
        
        nodes_in_row = []
        
        if layer_type == "even":
        
            #for col, pt in enumerate(self.get_pts_in_row_rgmesh_fab(row, srf=srf)):
            for col in range(2, self.cols):
                if col==0: # lead in nodes
                    pass
                elif col==self.cols: # lead out nodes
                    pass 
                else:
                    if col % 2 == 0: #and col >= 2: # just take every 3rd plane
                        node = self.generate_node_exterior(row, col, node_type = node_type, srf = srf, insert_discrete_element = True, layer_type = layer_type)
                        nodes_in_row.append(node)
        
        else: #layer_type == "odd":
            #for col, pt in enumerate(self.get_pts_in_row_rgmesh_fab(row, srf=srf)):
            for col in range(2, self.cols):
                if col==0: # lead in nodes
                    pass
                elif col==self.cols: # lead out nodes
                    pass 
                else:
                    if col % 2 == 1: #and col >= 2: # just take every 3rd plane
                        node = self.generate_node_exterior(row, col, node_type = node_type, srf = srf, insert_discrete_element = True, layer_type = layer_type)
                        nodes_in_row.append(node)
        
        self.calc_lines_in_cont_dir_row(nodes_in_row) # calculate the continuous members
        self.calc_bending_angles_row(nodes_in_row) # calculate the absolute bending angles of the individual nodes
        
        return nodes_in_row
    
    def calc_lines_in_cont_dir_row(self, nodes_in_row):
        '''this methods takes a row of nodes and calculates the line members in contiuous direction '''

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