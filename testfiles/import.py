'''
Created on 21.11.2017

@author: wd
'''

import json

def to_matrix(l, n):
    return [l[i:i+n] for i in xrange(0, len(l), n)]

def import_from_txt(file_name):
        
    f = open(file_name, 'r')
    
    vertices_3darray = []  
    row = []
    for s in f.readlines():
        if len(s)>1:
            row.append(to_matrix(json.loads(s), 3))
        else:
            vertices_3darray.append(row)
            row = []
    return vertices_3darray


vertices_3darray = import_from_txt('export.txt')
print "first point in cell: ", vertices_3darray[0][0][0]


"""

[500.0, -222.60627746582031, 612.39935302734375, 485.65426635742187, -246.05120849609375, 637.4459228515625, 486.10256958007812, -243.88810729980469, 781.31890869140625, 500.0, -220.28868103027344, 762.39935302734375]
1 line = [p0_x, p0_y, p0_z, p1_x, p1_y, p1_z, p2_x, p2_y, p2_z, p3_x, p3_y,p3_z]
empty line = next row

"""
