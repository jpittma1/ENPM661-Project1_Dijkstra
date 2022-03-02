#!/usr/bin/env python3

#ENPM673 Spring 2022
#Section 0101
#Jerry Pittman, Jr. UID: 117707120
#jpittma1@umd.edu
#Project #2 Obstacles

import numpy as np
from numpy import linalg as LA

sizex = 400
sizey = 250
robot_radius = 0
clearance = 5
total_clearance = robot_radius + clearance

#Boomerang shape==2 Triangles
b_bottom_x=105
b_bottom_y=100
b_top_x=115
b_top_y=210

b_middle_left_x=36
b_middle_right_x=80
b_middle_left_y=185
b_middle_right_y=180

left_x=b_middle_left_x-total_clearance
left_y=b_middle_left_y
triangle_top_x=b_top_x+total_clearance
triangle_top_y=b_top_y+total_clearance
right_x=b_middle_right_x+total_clearance
right_y=b_middle_right_y
triangle_bottom_x=b_bottom_x-total_clearance
triangle_bottom_y=b_bottom_y-total_clearance

#----Top Triangle of Boomerang--------
boomerang_pts_top=np.array([[right_x, right_y],
                        [triangle_top_x,triangle_top_y],
                        [left_x,left_y]], np.int32)

#----Bottom Triangle of Boomerang---------
boomerang_pts_bottom=np.array([[left_x,left_y],
                        [triangle_bottom_x,triangle_bottom_y],
                        [right_x, right_y]], np.int32)


#circle values from map
circle_diameter = 80 
circle_offset_x = 300 #400-100
circle_offset_y = 185 #250-65
circle_radius = int(circle_diameter/2 + total_clearance)

#hexagon values from map
hexagon_diameter=70
hexagon_radius=int(hexagon_diameter/2+total_clearance)

hexagon_offset_x=200
hexagon_offset_y=100
hexagon_left_x= hexagon_offset_x-hexagon_radius
hexagon_right_x= hexagon_offset_x+hexagon_radius

hexagon_r = int(hexagon_diameter/2)
hexagon_corner=int(hexagon_diameter/4)
hexagon_left_x=hexagon_offset_x-hexagon_r-total_clearance
hexagon_upper_y=hexagon_offset_y+hexagon_corner
hexagon_lower_y=hexagon_offset_y-hexagon_corner
hexagon_right_x=hexagon_offset_x+hexagon_r+total_clearance
hexagon_top_x=hexagon_offset_x
hexagon_top_y=hexagon_offset_y+hexagon_r+total_clearance
hexagon_bottom_x=hexagon_top_x
hexagon_bottom_y=hexagon_offset_y-hexagon_r-total_clearance

hexagon_pts=np.array([[hexagon_left_x,hexagon_upper_y],
                    [hexagon_top_x,hexagon_top_y],
                    [hexagon_right_x,hexagon_upper_y],
                    [hexagon_right_x,hexagon_lower_y],
                    [hexagon_bottom_x,hexagon_bottom_y],
                    [hexagon_left_x,hexagon_lower_y]], np.int32)

# hexagon_pts=np.array([[hexagon_offset_x,hexagon_offset_y-hexagon_radius-total_clearance],
#                     [hexagon_right_x+total_clearance, hexagon_offset_y-hexagon_radius/2],
#                     [hexagon_right_x+total_clearance, hexagon_offset_y+hexagon_radius/2],
#                     [hexagon_offset_x,hexagon_offset_y+hexagon_radius+total_clearance],
#                     [hexagon_left_x-total_clearance,hexagon_offset_y+hexagon_radius/2],
#                     [hexagon_left_x-total_clearance,hexagon_offset_y-hexagon_radius/2]], np.int32)


# print("hexagon polypoints", hexagon_pts)
# hexagon_pts = hexagon_pts.reshape((-1,1,2))