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

Created on 14.01.2017

@author: kathrind
'''

import Rhino.Geometry as rg
import ghpythonlib.components as ghcomp
import math as m
from System.Collections.Generic import IEnumerable, List


class MMMesh: 
    
    #===========================================================================
    # MM Mesh
    #===========================================================================
    def __init__(self):
        ''' generation geometry of a mesh mould mesh '''
   
        # maybe later replace with u and v     
        self.rows = 0 # is the division in the discrete direction # horizontal division along the motion path #rows
        self.cols = 0 # is the division in the continuous direction * 2 for subdivision # vertical division #cols
        
        # deriving from the init mesehs we generate rebuilt meshes that are rebuilt from the intersection curves/points       
        self.rgmesh_fab_srf1 = rg.Mesh()
        self.rgmesh_fab_srf2 = rg.Mesh()
        
        # cutting planes
        self.cutting_planes = [] # these planes are also for the wrist direction of the robot arm of each layer 
    
    #===========================================================================
    # setting the fab meshes with number of rows, cols and cutting planes
    #===========================================================================
    def set_rgmeshes_fab(self, rgmesh_fab_srf1, rgmesh_fab_srf2, cutting_planes, rows, cols):
        
        # rhino mesh constructed with cutting planes
        self.rgmesh_fab_srf1 = rgmesh_fab_srf1
        self.rgmesh_fab_srf2 = rgmesh_fab_srf2
        
        # cutting planes
        self.cutting_planes = cutting_planes # these planes are also for the wrist direction of the robot arm of each layer
        
        # number of rows and cols
        self.rows = rows # is the division in the discrete direction # horizontal division along the motion path #rows
        self.cols = cols # is the division in the continuous direction * 2 for subdivision # vertical division #cols

    #===========================================================================
    # functions for generating the regular quadmesh based on init meshes and the cutting planes
    #===========================================================================     
    def generate_rgmeshes_fab(self, init_rgmesh_srf1, init_rgmesh_srf2, crv_for_cutting_planes, cell_width, cell_length, min_intersection_crv_len = 2800, split_plane = None):
        ''' generate the rhino fab meshes from  init meshes and cutting planes '''
        
        self.init_rgmesh_srf1 = init_rgmesh_srf1
        self.init_rgmesh_srf2 = init_rgmesh_srf2
        
        # cutting planes for surface
        self.init_cutting_planes = self.get_init_cutting_planes_from_crv(crv_for_cutting_planes, cell_width) # these planes are also for the wrist direction of the robot arm of each layer
        
        # division between discrete elements of the mesh
        self.cell_length = cell_length
        
        # create the intersection planes from the surfaces with the cutting planes
        self.create_intersection_curves_from_init_meshes(min_intersection_crv_len = min_intersection_crv_len, split_plane = split_plane)
        
        # create the division points from the intersection lines
        self.generate_intersection_pts_in_layers(self.cell_length)
        
        self.rgmesh_fab_srf1 = self.generate_rgmesh_from_pts_in_rows(self.intersection_pts_in_layers_srf1)
        self.rgmesh_fab_srf2 = self.generate_rgmesh_from_pts_in_rows(self.intersection_pts_in_layers_srf2)
    
    def get_init_cutting_planes_from_crv(self, crv_for_cutting_planes, cell_width):
        # get intersection curves of the srfs with the perpendicular plane to the mean curve       
        div_num = self.get_crv_div_num(crv_for_cutting_planes, cell_width*1.01)
        cutting_planes = ghcomp.PerpFrames(crv_for_cutting_planes, div_num)[0]
        [p.Rotate(m.radians(90), p.ZAxis, p.Origin) for p in cutting_planes]
        
        return cutting_planes
    
    def generate_intersection_pts_in_layers(self, cell_length):
        ''' create a grid of points deriving from the intersection curves, the division distance is depending on the desired cell length times 3 '''        
        
        divnum = self.get_crv_div_num(self.intersection_crvs_srf1[0], cell_length) * 2
        
        self.intersection_pts_in_layers_srf1 = self.get_divpts_from_crvs_in_layers(self.intersection_crvs_srf1, divnum + 1)
        self.intersection_pts_in_layers_srf2 = self.get_divpts_from_crvs_in_layers(self.intersection_crvs_srf2, divnum + 1)
        
        self.rows = len(self.intersection_pts_in_layers_srf1) #rows
        self.cols = len(self.intersection_pts_in_layers_srf1[0]) #cols

    def create_intersection_curves_from_init_meshes(self, min_intersection_crv_len = 2800, split_plane = None):   
        ''' the two surfaces are cut by a list of cutting planes, which result in a list of intersection curves '''

        self.intersection_crvs_srf1 = []
        self.intersection_crvs_srf2 = []
        self.cutting_planes = []

        for cplane in self.init_cutting_planes:
            crv_srf1 = self.create_intersect_curve_mesh_with_plane(self.init_rgmesh_srf1, cplane, min_intersection_crv_len = min_intersection_crv_len, return_nurbs_curves = True)
            crv_srf2 = self.create_intersect_curve_mesh_with_plane(self.init_rgmesh_srf2, cplane, min_intersection_crv_len = min_intersection_crv_len, return_nurbs_curves = True)

            if crv_srf1 and crv_srf2:
                
                if split_plane:
                    ie_srf1 = rg.Intersect.Intersection.CurvePlane(crv_srf1, split_plane, 0.1)[0]
                    ie_srf2 = rg.Intersect.Intersection.CurvePlane(crv_srf2, split_plane, 0.1)[0]
                    
                    split_crv_srf1 = crv_srf1.Split(ie_srf1.ParameterA)[1]
                    split_crv_srf2 = crv_srf2.Split(ie_srf2.ParameterA)[1]
                    
                    crv_srf1 = split_crv_srf1
                    crv_srf2 = split_crv_srf2
                    
                self.intersection_crvs_srf1.append(crv_srf1)
                self.intersection_crvs_srf2.append(crv_srf2)
                
                self.cutting_planes.append(cplane) # these are also the wrist planes for the robot
            
    def get_intersection_pts_flat(self, srf = "srf1"):     
        if srf == "srf1":
            return [x for sublist in self.intersection_pts_in_layers_srf1 for x in sublist]
        else:
            return [x for sublist in self.intersection_pts_in_layers_srf2 for x in sublist]

    def get_pt_in_row_intersection_pts(self, row, col, srf = "srf1"):
        '''return the point of the point grid given column and row and surface 1 or surface 2'''    
        return self.intersection_pts_in_layers_srf1[row][col] if srf == "srf1" else self.intersection_pts_in_layers_srf2[row][col]

    def get_crv_div_num(self, crv, div_len):
        ''' returns the number of divisions for a curve given a length '''
        length = ghcomp.Length(crv) #length = polyline.Length
        div_num = int((length / div_len))
        return div_num

    def get_min_dist_crv_to_plane(self, plane, crvs):
        ''' returns a curve out of a list of curves which midpoint is closest to a given plane '''
        min_dist = 10000000
        min_crv = None
        for crv in crvs:
            midpt = crv.PointAt(0.5)
            dist = plane.Origin.DistanceTo(midpt)
            if dist < min_dist:
                min_dist = dist
                min_crv = crv            
        return min_crv
    
    def get_plane_from_srf(self, pt, srf):
        '''get the srf plane closest tot the input point'''
        
        s = self.srf1_rebuilt if srf == 1 else self.srf2_rebuilt
        
        srf_param = s.ClosestPoint(pt)
        srf_plane = s.FrameAt(srf_param[1], srf_param[2])[1]
        
        return srf_plane
    
    def create_intersect_curve_mesh_with_plane(self, mesh, cutting_plane, min_intersection_crv_len = 2300, return_nurbs_curves = True):   
        ''' a mesh is cut by plane, the function returns the curve closest to the cutting plane, threshold for min length must be given '''
      
        intersection_crvs = rg.Intersect.Intersection.MeshPlane(mesh, cutting_plane)
        #print intersection_crvs
        if intersection_crvs:
            if len(intersection_crvs) == 1:         
                crv = intersection_crvs[0]       
            elif len(intersection_crvs) > 1:            
                crv = self.get_min_dist_crv_to_plane(cutting_plane, intersection_crvs)                   
            if crv.Length > min_intersection_crv_len:           
                if return_nurbs_curves: crv = crv.ToNurbsCurve()
                if crv.PointAtStart.Z > crv.PointAtEnd.Z:
                    crv.Reverse()            
                return crv
            else:
                return None
        else:
            return None
            
    def get_loft_srf_rebuilt_from_crvs(self, crvs, resolution = 50):
        ''' return srf rebuilt from crvs''' 
                
        ielist_crvs = List[rg.Curve]()
        [ielist_crvs.Add(crv) for crv in crvs]
    
        # create a new loft with the curves
        srf_rebuilt = rg.Brep.CreateFromLoftRebuild(ielist_crvs, rg.Point3d.Unset, rg.Point3d.Unset, rg.LoftType.Normal, False, resolution)[0]    
        return srf_rebuilt.Faces[0].ToNurbsSurface()
    
    def get_loft_srf_from_crvs(self, crvs):
        ''' return srf rebuilt from crvs''' 
                
        ielist_crvs = List[rg.Curve]()
        [ielist_crvs.Add(crv) for crv in crvs]
    
        # create a new loft with the curves
        srf_rebuilt = rg.Brep.CreateFromLoft(ielist_crvs, rg.Point3d.Unset, rg.Point3d.Unset, rg.LoftType.Normal, False)[0]    
        return srf_rebuilt.Faces[0].ToNurbsSurface()
    
    def generate_rgmesh_from_pts(self, pts, rows, cols):
        ''' generate a rhino mesh out of pts and numbers of division in both direction '''
        
        rgmesh = rg.Mesh()
        
        # adding the mesh vertices
        [rgmesh.Vertices.Add(pt) for pt in pts]
        
        # adding the mesh faces
        for j in range(rows):
            for i in range(cols):
                current_idx = j * cols + i
                current_idx_row_before = current_idx - cols
                if j > 0 and i < cols-1:
                    rgmesh.Faces.AddFace(current_idx, current_idx+1, current_idx_row_before+1, current_idx_row_before)              
        return rgmesh
    
    def get_flat_list_from_2dlist(self, nested):
        ''' returns a flat list from a 2d nested list'''     
        return [x for sublist in nested for x in sublist]
    
    def generate_rgmesh_from_pts_in_rows(self, pts_in_rows):
        ''' generate a rhino mesh out of pts and numbers of division in both direction '''
        rows = len(pts_in_rows)
        cols = len(pts_in_rows[0])
        
        return self.generate_rgmesh_from_pts(self.get_flat_list_from_2dlist(pts_in_rows), rows, cols)
    
    def get_divpts_from_crv(self, crv, divnum):
        ''' return points deriving from the division of a curve'''
        params = crv.DivideByCount(divnum, True)
        return [crv.PointAt(p) for p in params]   
    
    def get_divpts_from_crvs_in_layers(self, crvs, divnum):
        ''' return division points in layers from crvs'''
        layers = []
        for crv in crvs:
            layers.append(self.get_divpts_from_crv(crv, divnum))
        return layers
    
    def get_vertices_in_row_rgmesh_fab(self, row, srf="srf1"):
        return self.get_vertices_in_row_rgmesh(self.rgmesh_fab_srf1, row, self.cols) if srf=="srf1" else self.get_vertices_in_row_rgmesh(self.rgmesh_fab_srf2, row, self.cols)
    
    def get_vertices_in_col_rgmesh_fab(self, col, srf="srf1"):
        return self.get_vertices_in_col_rgmesh(self.rgmesh_fab_srf1, col, self.rows, self.cols) if srf=="srf1" else self.get_vertices_in_col_rgmesh(self.rgmesh_fab_srf2, col, self.rows, self.cols)

    def get_pts_in_row_rgmesh_fab(self, row, srf="srf1"):
        return self.get_pts_in_row_rgmesh(self.rgmesh_fab_srf1, row, self.cols) if srf=="srf1" else self.get_pts_in_row_rgmesh(self.rgmesh_fab_srf2, row, self.cols)
    
    def get_pts_in_col_rgmesh_fab(self, col, srf="srf1"):
        return self.get_pts_in_col_rgmesh(self.rgmesh_fab_srf1, col, self.rows, self.cols) if srf=="srf1" else self.get_pts_in_col_rgmesh(self.rgmesh_fab_srf2, col, self.rows, self.cols)
    
    def get_pt_row_col_rgmesh_fab(self, row, col, srf = "srf1"):
        return self.get_pt_row_col_rgmesh(self.rgmesh_fab_srf1, row, col, self.cols) if srf=="srf1" else self.get_pt_row_col_rgmesh(self.rgmesh_fab_srf2, row, col, self.cols)

    def get_edge_lines_in_row_rgmesh_fab(self, row, srf = "srf1"):
        return self.get_edge_lines_in_row_rgmesh(self.rgmesh_fab_srf1, row, self.cols) if srf=="srf1" else self.get_edge_lines_in_row_rgmesh(self.rgmesh_fab_srf2, row, self.cols)
    
    def get_edge_lines_in_col_rgmesh_fab(self, col, srf = "srf1"):
        return self.get_edge_lines_in_col_rgmesh(self.rgmesh_fab_srf1, col, self.rows, self.cols) if srf=="srf1" else self.get_edge_lines_in_col_rgmesh(self.rgmesh_fab_srf2, col, self.rows, self.cols)
    
    def get_edge_lines_col_rgmesh_fab(self, srf = "srf1"):
        return self.get_edge_lines_col_rgmesh(self.rgmesh_fab_srf1, self.rows, self.cols) if srf=="srf1" else self.get_edge_lines_col_rgmesh(self.rgmesh_fab_srf2, self.rows, self.cols)
    
    def get_edge_lines_row_rgmesh_fab(self, srf = "srf1"):
        return self.get_edge_lines_row_rgmesh(self.rgmesh_fab_srf1, self.rows, self.cols) if srf=="srf1" else self.get_edge_lines_row_rgmesh(self.rgmesh_fab_srf2, self.rows, self.cols)

    
    #===========================================================================
    # general rg mesh functions for a quadmesh with regular grid
    #===========================================================================
    def get_idx_row_col(self, row, col, cols):
        ''' return idx in row col from a regular grid'''
        return row * cols + col
    
    def get_vertex_row_col_rgmesh(self, rgmesh, row, col, cols):
        ''' return vertex in row col from a regular grid'''
        return rgmesh.Vertices[self.get_idx_row_col(row, col, cols)]
    
    def get_pt_row_col_rgmesh(self, rgmesh, row, col, cols):
        ''' return pt in row col from a regular grid'''
        #print row, col, cols
        return rg.Point3d(rgmesh.Vertices[self.get_idx_row_col(row, col, cols)])
    
    def get_idcs_in_row(self, row, cols):
        ''' return idcs in row from a regular grid'''
        from_idx = row * cols
        to_idx = (row + 1) * cols
        return range(from_idx, to_idx)
    
    def get_idcs_in_col(self, col, rows, cols):
        ''' return idcs in col from a regular grid'''
        return [(i * cols) + col for i in range(rows)]
    
    def get_vertices_in_row_rgmesh(self, rgmesh, row, cols):
        ''' return vertices in row from a rhino geometry quad mesh'''   
        return [rgmesh.Vertices[idx] for idx in self.get_idcs_in_row(row, cols)]
                
    def get_vertices_in_col_rgmesh(self, rgmesh, col, rows, cols):
        ''' return vertices in col from a rhino geometry quad mesh'''   
        return [rgmesh.Vertices[idx] for idx in self.get_idcs_in_col(col, rows, cols)]
        
    def get_pts_in_row_rgmesh(self, rgmesh, row, cols):
        ''' return pts in row from a rhino geometry quad mesh'''  
        return [rg.Point3d(rgmesh.Vertices[idx]) for idx in self.get_idcs_in_row(row, cols)]
    
    def get_pts_in_col_rgmesh(self, rgmesh, col, rows, cols):
        ''' return pts in col from a rhino geometry quad mesh'''  
        return [rg.Point3d(rgmesh.Vertices[idx]) for idx in self.get_idcs_in_col(col, rows, cols)]
        
    def get_edge_lines_in_row_rgmesh(self, rgmesh, row, cols):
    
        idcs_vertices = self.get_idcs_in_row(row, cols)
        lines = []
        
        for i, idx in enumerate(idcs_vertices):
            if i>0:
                idx_start = idcs_vertices[i-1]
                idx_end = idcs_vertices[i]
                line = rg.Line(rgmesh.Vertices[idx_start],rgmesh.Vertices[idx_end]) 
                lines.append(line)
        
        return lines
    
    def get_edge_lines_in_col_rgmesh(self, rgmesh, col, rows, cols):
        
        idcs_vertices = self.get_idcs_in_col(col, rows, cols)
        lines = []
        #print idcs_vertices
        
        for i, idx in enumerate(idcs_vertices):
            if i>0:
                idx_start = idcs_vertices[i-1]
                idx_end = idcs_vertices[i]            
                line = rg.Line(rgmesh.Vertices[idx_start],rgmesh.Vertices[idx_end]) 
                lines.append(line)
        return lines

    def get_edge_lines_col_rgmesh(self, rgmesh, rows, cols):
        lines = []
        for col in range(cols):
            lines += self.get_edge_lines_in_col_rgmesh(rgmesh, col, rows, cols)
        return lines
    
    def get_edge_lines_row_rgmesh(self, rgmesh, rows, cols):
        lines = []
        for row in range(rows):
            lines += self.get_edge_lines_in_row_rgmesh(rgmesh, row, cols)
        return lines
    
    def get_length_of_longest_row_rgmesh(self, rgmesh, rows, cols):
        max_len = 0
        for row in range(rows):
            lines = self.get_edge_lines_in_row_rgmesh(rgmesh, row, cols)
            total_len = sum([l.Length for l in lines])
            if total_len > max_len:
                max_len = total_len
        return max_len
        
    def get_min_max_line_length(self, lines):
        lengths = [l.Length for l in lines]
        return (min(lengths), max(lengths))
    
    def get_min_max_lines(self, lines):
        lengths = [l.Length for l in lines]
        minv, maxv = self.get_min_max_line_length(lines)
        #print minv, maxv
        return (lines[lengths.index(minv)], lines[lengths.index(maxv)])
    
    def get_lines_outside_lengths_thresh(self, lines, min_len, max_len):
        return [l for l in lines if l.Length<min_len or l.Length>max_len]
        """
        lines = self.get_edge_lines_col_rgmesh(rgmesh, rows, cols)
        lengths = [l.Length for l in lines]
        return sum(lengths)"""
    
    def get_sum_of_edge_lengths_col_with_excess_length_rgmesh(self, rgmesh, rows, cols, elength=30):
        lines = self.get_edge_lines_col_rgmesh(rgmesh, rows, cols)
        lengths = [l.Length+elength for i, l in enumerate(lines) if i%2==0]
        return sum(lengths)
    
    def get_sum_of_edge_lengths_row_with_excess_length_rgmesh(self, rgmesh, rows, cols, elength=600):
        # for each row add 500mm on the bottom and 100mm on top
        lines = self.get_edge_lines_row_rgmesh(rgmesh, rows, cols)
        lengths = [l.Length for l in lines]
        add_length = rows * 600
        return sum(lengths) + add_length
    
    def get_rgmesh_from_subdomain(self, uv_subdomain):
        row_from, row_to, col_from, col_to = (int(uv_subdomain.U0), int(uv_subdomain.U1), int(uv_subdomain.V0), int(uv_subdomain.V1))
        
        row_to = min(self.rows, row_to)
        col_to = min(self.cols, col_to)
        
        cutting_planes = self.cutting_planes[row_from:row_to]
        
        pts_in_rows_srf1 = [self.get_pts_in_row_rgmesh_fab(row, srf="srf1")[col_from:col_to] for row in range(row_from, row_to)]
        pts_in_rows_srf2 = [self.get_pts_in_row_rgmesh_fab(row, srf="srf2")[col_from:col_to] for row in range(row_from, row_to)]
        
        rgmesh_fab_srf1 = self.generate_rgmesh_from_pts_in_rows(pts_in_rows_srf1)
        rgmesh_fab_srf2 = self.generate_rgmesh_from_pts_in_rows(pts_in_rows_srf2)
        
        return (cutting_planes, (rgmesh_fab_srf1, rgmesh_fab_srf2), len(range(row_from,row_to)), len(range(col_from,col_to)))
