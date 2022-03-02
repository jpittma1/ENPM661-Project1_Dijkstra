#!/usr/bin/env python3

#ENPM673 Spring 2022
#Section 0101
#Jerry Pittman, Jr. UID: 117707120
#jpittma1@umd.edu
#Project #2 Obstacles

import numpy as np
from numpy import linalg as LA

import copy
import timeit
import queue
from queue import PriorityQueue
import cv2
import scipy
from scipy import fft, ifft
import matplotlib.pyplot as plt
import sys
import math


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

#Points of the boomerang, add clearance on right(+x), top(+y), bottom(-y), and left (-x)
#Split into two triangles; CCW

boomerang_pts_A=np.array([[b_middle_left_x-total_clearance,b_middle_left_y],
                        [b_bottom_x-total_clearance, b_bottom_y-total_clearance],
                        [b_middle_right_x+total_clearance,b_middle_right_y]], np.int32)
#Top
boomerang_pts_B=np.array([[b_middle_right_x+total_clearance,b_middle_left_y],
                        [b_top_x+total_clearance,b_top_y+total_clearance],
                        [b_middle_left_x-total_clearance,b_middle_left_y]], np.int32)

# boomerang_pts_A = boomerang_pts_A.reshape((-1,1,2))
# boomerang_pts_B = boomerang_pts_B.reshape((-1,1,2))

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

hexagon_pts=np.array([[hexagon_offset_x,hexagon_offset_y-hexagon_radius-total_clearance],
                    [hexagon_right_x+total_clearance, hexagon_offset_y-hexagon_radius/2],
                    [hexagon_right_x+total_clearance, hexagon_offset_y+hexagon_radius/2],
                    [hexagon_offset_x,hexagon_offset_y+hexagon_radius+total_clearance],
                    [hexagon_left_x-total_clearance,hexagon_offset_y+hexagon_radius/2],
                    [hexagon_left_x-total_clearance,hexagon_offset_y-hexagon_radius/2]], np.int32)


# print("hexagon polypoints", hexagon_pts)
# hexagon_pts = hexagon_pts.reshape((-1,1,2))

def updateNodesOnMap(map, node_state, color):
    x,y, _ = map.shape
    trans_y = node_state[0]  
    trans_x = x - node_state[1] - 1
    map[trans_x,trans_y, :] = color
    
    return map

#Equation of line for Hexagon and Boomerang
def lineEquation(p1,p2,x,y):
    func = ((p2[1] - p1[1]) * (x - p1[0])) / ( p2[0] - p1[0]) + p1[1] - y
    
    return func

def addObstacles2Map(map):
    
    #########---------PLOT Circle----------------#########
    for i in range(circle_offset_x - circle_radius, circle_offset_x + circle_radius):
        for j in range(circle_offset_y - circle_radius, circle_offset_y + circle_radius):
            if (i - circle_offset_x) **2 + (j - circle_offset_y)**2 <= circle_radius**2:
                updateNodesOnMap(map, [i, j], [0,255,255])
    

    for i in range(map.shape[1]):
       for j in range(map.shape[0]):
            #-----HEXAGON--------------------------
            if (i<hexagon_right_x and i>hexagon_left_x and lineEquation((hexagon_left_x,hexagon_upper_y),(hexagon_top_x,hexagon_top_y),i,j) > 0 and lineEquation((hexagon_top_x,hexagon_top_y),(hexagon_right_x,hexagon_upper_y),i,j) > 0 and lineEquation((hexagon_left_x,hexagon_lower_y),(hexagon_bottom_x,hexagon_bottom_y),i,j) < 0 and lineEquation((hexagon_bottom_x,hexagon_bottom_y),(hexagon_right_x,hexagon_lower_y),i,j) < 0):
                updateNodesOnMap(map, [i, j], [0,255,255])
            
            #----Top Triangle of Boomerang--------
            if(lineEquation((left_x,left_y),(triangle_top_x,triangle_top_y),i,j) >0 and lineEquation((triangle_top_x,triangle_top_y),(right_x, right_y),i,j) <0 and lineEquation((left_x,left_y),(right_x, right_y),i,j) <0):
                updateNodesOnMap(map, [i, j], [0,255,255])

            #----Bottom Triangle of Boomerang---------
            if(lineEquation((left_x,left_y),(triangle_bottom_x,triangle_bottom_y),i,j) <0 and lineEquation((triangle_bottom_x,triangle_bottom_y),(right_x, right_y),i,j) >0 and lineEquation((right_x, right_y),(left_x,left_y),i,j) >0):
                updateNodesOnMap(map, [i, j], [0,255,255])
            
            
    return map

Xi = [0,0] #starts at Origin
# Xg=[380, 20] #above hexagon
Xg=[200, 210] #above hexagon
# Xg=[390,240] #behind circle

map_size = [250, 400] 
map_y, map_x = map_size

space = np.zeros([map_size[0], map_size[1], 3], dtype=np.uint8) 

# print("space shape", space.shape)
space = updateNodesOnMap(space, Xi, [255,255,0])
space = updateNodesOnMap(space, Xg, [255,255,0])
space = addObstacles2Map(space)

cv2.namedWindow("map", cv2.WINDOW_NORMAL)
cv2.imwrite('Initial_map.jpg', space)