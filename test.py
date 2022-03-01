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

#boomerang shape
'''Going CCW, hessian normal form.
V1=bottom to middle_right
V2=middle_right to top
V3=top to middle_left
V4=middle_left to bottom
'''
b_bottom_x=105
b_bottom_y=100
b_top_x=210
b_top_y=115

b_middle_left_x=36
b_middle_right_x=80
b_middle_y=185

#Points of the boomerang, add clearance on right(+x), top(+y), bottom(-y), and left (-x)
#Split into two triangles; CCW
# boomerang_pts_A=np.array([[b_middle_left_x-total_clearance,b_middle_y],
#                         [b_bottom_x, b_bottom_y-total_clearance],
#                         [b_middle_right_x+total_clearance,b_middle_y]], np.int32)

# boomerang_pts_B=np.array([[b_middle_right_x+total_clearance,b_middle_y],
#                         [b_top_x,b_top_y+total_clearance],
#                         [b_middle_left_x-total_clearance,b_middle_y]], np.int32)

boomerang_pts_A=np.array([[b_middle_left_x,b_middle_y],
                        [b_bottom_x, b_bottom_y],
                        [b_middle_right_x,b_middle_y]], np.int32)

boomerang_pts_B=np.array([[b_middle_right_x,b_middle_y],
                        [b_top_x,b_top_y],
                        [b_middle_left_x,b_middle_y]], np.int32)

# print("Boomerang polypoints", boomerang_pts)
boomerang_pts_A = boomerang_pts_A.reshape((-1,1,2))
boomerang_pts_B = boomerang_pts_A.reshape((-1,1,2))
# print("Boomerang polypoints", boomerang_pts)

# pts = np.array([[10,5],[20,30],[70,20],[50,10]], np.int32)
# pts = pts.reshape((-1,1,2))
# cv.polylines(img,[pts],True,(0,255,255))
# result = cv2.pointPolygonTest(contour, (x,y), False) 
# positive (inside), negative (outside), or zero (on an edge) value,
# In the function, the third argument is measureDist. If it is True, it finds the shortest distance between a point in the image and a contour. If False, it finds whether the point is inside, 
# outside, or on the contour. Since we don't want to find the distance, we set the measureDist argument to False


# black_frame = np.zeros_like(your_frame).astype(np.uint8)
# cv2.fillPoly(black_frame , [hull], (255, 255, 255))

# V1=bottom to middle_right
# V1_x = b_middle_right_x - b_bottom_x
# V1_y = b_middle_y - b_bottom_y

# # V2=middle_right to top
# V2_x = b_top_x - b_middle_right_x
# V2_y = b_top_y - b_middle_y

# # V3=top to middle_left
# V3_x = b_middle_left_x - b_top_x
# V3_y = b_middle_y - b_top_y

# # V4=middle_left to bottom
# V4_x = b_bottom_x - b_middle_left_x
# V4_y = b_bottom_y - b_middle_y

# #N_hat values
# tmp=np.array([[V1_y],[-V1_x]])
# N_hat1=tmp/np.linalg.norm(tmp)
# # print(N_hat1)
# tmp=np.array([[V2_y],[-V2_x]])
# N_hat2=tmp/np.linalg.norm(tmp)
# tmp=np.array([[V3_y],[-V3_x]])
# N_hat3=tmp/np.linalg.norm(tmp)
# tmp=np.array([[V4_y],[-V4_x]])
# N_hat4=tmp/np.linalg.norm(tmp)

#N_hat*x=-p


#eqn boomerang top is y=0.316x+16.26

#circle
circle_diameter = 80 
circle_offset_x = 300 #400-100
circle_offset_y = 185 #250-65
circle_radius = int(circle_diameter/2 + total_clearance)

#hexagon
hexagon_diameter=70
hexagon_radius = hexagon_diameter/2

hexagon_offset_x=200
hexagon_offset_y=100
hexagon_left_x= hexagon_offset_x-hexagon_radius
hexagon_right_x= hexagon_offset_x+hexagon_radius

hexagon_pts=np.array([[hexagon_offset_x,hexagon_offset_y-hexagon_radius-total_clearance],
                    [hexagon_right_x+total_clearance, hexagon_offset_y-hexagon_radius/2],
                    [hexagon_right_x+total_clearance, hexagon_offset_y+hexagon_radius/2],
                    [hexagon_offset_x,hexagon_offset_y+hexagon_radius+total_clearance],
                    [hexagon_left_x-total_clearance,hexagon_offset_y+hexagon_radius/2],
                    [hexagon_left_x-total_clearance,hexagon_offset_y-hexagon_radius/2]], np.int32)


# print("hexagon polypoints", hexagon_pts)
hexagon_pts = hexagon_pts.reshape((-1,1,2))

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
    # cv2.circle(map, (circle_offset_x, circle_offset_y),circle_radius, (0,255,255),-1)
    # cv2.circle(map, (300, 185),45, (0,255,255),-1)
    # map=cv2.circle(map, [300,185],circle_radius, (250,0,0),3)
    # cv2.circle(map, [300,185],circle_radius, (255,0,0),3)
    # circle_diameter = 80 
    # circle_offset_x = 300 #400-100
    # circle_offset_y = 185 #250-65
    # circle_radius = int(circle_diameter/2 + total_clearance)
    for i in range(circle_offset_x - circle_radius, circle_offset_x + circle_radius):
        for j in range(circle_offset_y - circle_radius, circle_offset_y + circle_radius):
            if (i - circle_offset_x) **2 + (j - circle_offset_y)**2 <= circle_radius**2:
                updateNodesOnMap(map, [i, j], [0,255,255])
    
    # hexagon_diameter=70
    # hexagon_radius = hexagon_diameter/2
    # hexagon_offset_x=200
    # hexagon_offset_y=100
    # b_bottom_x=105
    # b_bottom_y=100
    # b_top_x=210
    # b_top_y=115
    # b_middle_left_x=36
    # b_middle_right_x=80
    # b_middle_y=185 
    # clearance = 5
    # total_clearance = robot_radius + clearance
    for i in range(map.shape[0]):
        for j in range(map.shape[1]):
            
            # ######--------------Boomerang Top-----------------#########
            # if (lineEquation((31,65),(120,35),i,j) < 0 and lineEquation((31,65),(105,155),i,j) > 0 and lineEquation((80,70),(110,155),i,j) < 0):
            #     updateNodesOnMap(map, [i, j], [0,255,255])
            
            ######--------------Hexagon-----------------#########
            if (i > 160 and i < 240 and lineEquation((160,130),(200,110),i,j) < 0 and lineEquation((200,110),(240,130),i,j) < 0 and lineEquation((160,170.20),(200,190),i,j) > 0 and lineEquation((200,190),(240,170),i,j) > 0):
                updateNodesOnMap(map, [i, j], [0,255,255])
                
            # #########---------Boomerang Bottom----------------#########
            # if (lineEquation((80,75),(110,155),i,j) > 0 and lineEquation((31,65),(120,35),i,j) < 0 and lineEquation((80,70),(120,35),i,j) > 0):
            #     updateNodesOnMap(map, [i, j], [0,255,255])
    
    # cv2.imshow(map)
    
    #########---------PLOT Boomerang----------------#########
    #-----Boomerang--------------
    # cv2.polylines(map,[boomerang_pts],True,(0,255,255))
    cv2.fillConvexPoly(map,boomerang_pts_A,(0,255,255))
    cv2.fillConvexPoly(map,boomerang_pts_B,(0,255,255))
    
    #----------Hexagon----------------
    # cv2.polylines(map,[hexagon_pts],True,(0,255,255))
    # cv2.fillConvexPoly(map,hexagon_pts,(255,0,0))
    cv2.fillConvexPoly(map,hexagon_pts,(0,255,255))
    
    # result = cv2.pointPolygonTest(contour, (x,y), False) 
    # positive (inside), negative (outside), or zero (on an edge) value,
    # In the function, the third argument is measureDist. If it is True, it finds the shortest distance between a point in the image and a contour. If False, it finds whether the point is inside, 
    # outside, or on the contour. Since we don't want to find the distance, we set the measureDist argument to False


    # black_frame = np.zeros_like(your_frame).astype(np.uint8)
    # cv2.fillPoly(black_frame , [hull], (255, 255, 255))
    # cv2.imwrite('map.jpg', map)
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

cv2.imwrite('Initial_map.jpg', space)