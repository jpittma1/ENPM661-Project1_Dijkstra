#!/usr/bin/env python3

#ENPM673 Spring 2022
#Section 0101
#Jerry Pittman, Jr. UID: 117707120
#jpittma1@umd.edu
#Project #2 Functions

import copy
import timeit
import queue
from queue import PriorityQueue
import numpy as np
import cv2
import scipy
from scipy import fft, ifft
from numpy import linalg as LA
import matplotlib.pyplot as plt
import sys
import math
from obstacles import *
from Node import *

def GetInitialStates():
    print("Enter initial node, separated by spaces: ")
    initial=[int(x) for x in input().split()]
    print("Enter goal node, separated by spaces: ")
    final=[int(x) for x in input().split()]
    return initial, final

###########################################
'''OpenCV/ Visualization Functions'''

#To fix the origin from top left to bottom right
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

############################################################
'''Return 1 if within an obstacle or outside of map'''
def isInObstacleSpace(x,y):
    x_max=400-1
    y_max=250-1
    '''positive (inside), negative (outside), or zero (on an edge) value,
    In the function, the third argument is measureDist. If it is True, it finds the
    shortest distance between a point in the image and a contour. If False, it finds
    whether the point is inside, outside, or on the contour. Since we don't want to
    find the distance, we set the measureDist argument to False'''
    
    #Check if within Map
    if (x > x_max or int(x)<0 or int(y)<0 or int(y)>y_max):
        return 1
    
    #Check if within circle
    in_circle=(x-circle_offset_x)**2+(y-circle_offset_y)**2
    if in_circle <= (circle_radius)**2:
        return 1
    
    #check if within Hexagon
    in_hexagon=cv2.pointPolygonTest(hexagon_pts, (x,y), False)
    if in_hexagon>0:
        return 1
  
    #check if within boomerang
    
    in_boomerang_top=cv2.pointPolygonTest(boomerang_pts_top, (x,y), False)
    if in_boomerang_top>0:
        return 1
    
    in_boomerang_bottom=cv2.pointPolygonTest(boomerang_pts_bottom, (x,y), False)
    if in_boomerang_bottom>0:
        return 1
    
    return 0

def possibleMoves(current_node):
    # i = int(current_node[0])
    # j=  int(current_node[1])
    i,j=current_node.getState()
    # print("current_node [0", current_node[0])
    moves = ['N','NE', 'E', 'SE', 'S', 'SW','W', 'NW']
    poss_moves = ['N','NE', 'E', 'SE', 'S', 'SW','W', 'NW']
    move_i = [i, i+1, i+1, i+1, i, i-1, i-1, i-1]
    move_j = [j+1, j+1, j, j-1, j-1, j-1, j, j+1]
    for move in range(len(moves)):
        if (isInObstacleSpace(move_i[move], move_j[move]) or current_node.getParentState() == [move_i[move], move_j[move]]):
            poss_moves.remove(moves[move])
    # print(final_moves)
    return poss_moves

# def determinePossibleMoves(node):
#     i, j = node[3]
#     possibleMoves=[]
    
#     possibleMoves.append(ActionMoveUp(i,j))
#     possibleMoves.append(ActionMoveUpRight(i,j))
#     possibleMoves.append(ActionMoveRight(i,j))
#     possibleMoves.append(ActionMoveDownRight(i,j))
#     possibleMoves.append(ActionMoveDown(i,j))
#     possibleMoves.append(ActionMoveDownLeft(i,j))
#     possibleMoves.append(ActionMoveLeft(i,j))
#     possibleMoves.append(ActionMoveUpLeft(i,j))
    
    
#     # moves = ['N','NE', 'E', 'SE', 'S', 'SW','W', 'NW']
#     # # final_moves = ['N','NE', 'E', 'SE', 'S', 'SW','W', 'NW']
#     # move_i = [i, i+1, i+1, i+1, i, i-1, i-1, i-1]
#     # move_j = [j+1, j+1, j, j-1, j-1, j-1, j, j+1]
    
#     for move in range(len(possibleMoves)):
#         #verify not in obstacle and within map boundaries and if possible move is a parent
        
#         # if (isInObstacleSpace(move_i[move], move_j[move]) or node[2] == [move_i[move], move_j[move]]):
#         if (isInObstacleSpace(possibleMoves[move][0], possibleMoves[move][1]) or node[2] == [possibleMoves[move][0], possibleMoves[move][1]]):
#             possibleMoves.remove(possibleMoves[move])
    
#     print(possibleMoves)
    
#     return possibleMoves

'''-----8 Subfunctions for actions-----
Action sets= {(1,0), (-1,0), (0,1), 
(0,-1), (1,1), (-1,1),(1,-1),(-1,-1)}'''
#moves_cost = {'N':1, 'NE':1.4, 'E':1, 'SE':1.4, 'S':1, 'SW':1.4, 'W':1, 'NW':1.4}
def ActionMove(node,direction):
    if direction == 'N':
        NewNode = node
        NewNode[1] = node[1] + 1
        NewNode[2] = node[2] + 1
        cost = 1
    elif direction == 'NE':
        NewNode = node
        NewNode[1] = node[1] + 1
        NewNode[0] = node[0] + 1
        NewNode[2] = node[2] + 1
        cost = 1.4
    elif direction == 'E':
        NewNode = node
        NewNode[0] = node[0] + 1
        NewNode[2] = node[2] + 1
        cost = 1
    elif direction == 'SE':
        NewNode = node
        NewNode[1] = node[1] - 1
        NewNode[0] = node[0] + 1
        NewNode[2] = node[2] + 1
        cost = 1.4
    elif direction == 'S':
        NewNode = node
        NewNode[1] = node[1] - 1
        NewNode[2] = node[2] + 1
        cost = 1
    elif direction == 'SW':
        NewNode = node
        NewNode[1] = node[1] - 1
        NewNode[0] = node[0] - 1
        NewNode[2] = node[2] + 1
        cost = 1.4
    elif direction == 'W':
        NewNode = node
        NewNode[0] = node[0] - 1
        NewNode[2] = node[2] + 1
        cost = 1
    elif direction == 'NW':
        NewNode = node
        NewNode[1] = node[1] + 1
        NewNode[0] = node[0] - 1
        NewNode[2] = node[2] + 1
        cost = 1.4
    else:
        NewNode = node
        cost = 0

    return NewNode, cost

def ActionMoveLeft(x,y):
    NewNode = [x-1 , y]
    
    return NewNode

def ActionMoveRight(x,y):
    NewNode = [x+1 , y]
    
    return NewNode

def ActionMoveUp(x,y):
    NewNode = [x , y+1]
    
    return NewNode

def ActionMoveDown(x,y):
    NewNode = [x , y-1]
    
    return NewNode

def ActionMoveUpRight(x,y):
    NewNode = [x+1 , y+1]
    
    return NewNode

def ActionMoveDownRight(x,y):
    NewNode = [x+2 , y-1]
    
    return NewNode

def ActionMoveDownLeft(x,y):
    NewNode = [x-1 , y-1]
    
    return NewNode

def ActionMoveUpLeft(x,y):
    NewNode = [x-1 , y+1]
    
    return NewNode

# def getCost(direction):
    
#     if direction==
#         return 1
#     elif direction==

'''Check for Goal Node Function'''
def compare2Goal(now,goal):
    if np.array_equal(now, goal) or now==goal:
        return True
    else:
        return False

'''backtracking function'''
def generate_path(start, end, pathTaken):
    global Parent_Node_Index_i
    global Node_Index_i
    
    temp_path = []
    temp_path.append(end)
    
    # pathTaken[j][0] is child Node
    # pathTaken[j][1] is parent Node
    
    c=2 #Next Node Index is 2
    for i in range(len(pathTaken)):
        Node_Index_i.append(c)      #increment child index
        
        for j in range(len(pathTaken)):

            #compare last node of temp_path to child of pathway
            if temp_path[i] == pathTaken[j][0]: #path vs child
                temp_path.append(pathTaken[j][1]) #add parent to path

                previousParent=Parent_Node_Index_i[-1] #last parent node index value
                # print("Previous parent ", previousParent)

                if pathTaken[j][1]!=pathTaken[j-1][1]:  #increment parent index if new parent
                    Parent_Node_Index_i.append(previousParent+1)
                else:
                    Parent_Node_Index_i.append(previousParent)
                # print("temp_path[i] is ", temp_path[i])
                break
            
        if temp_path[i]==start: #determine if parent is start
            break
        c+=1
    
    # print("pre-reversed_path is ", temp_path)
    # print("Parent index is ", Parent_Node_Index_i)
    # print("Node index is ", Node_Index_i)
    
    path=[]
    #reverse path so goes start to goal
    for i in reversed(temp_path):
        path.append(i)
    
    # print("reversed path is ", path)
    # print("length of path is ", len(path))
    
    return path
    


def makeFiles(visited, last, path, p_index, n_index):
    '''Visited=Visited_Nodes, last=results
    path=pathway, p_index=Parent_Node_Index_i, n_index=Node_Index_i'''
    
    #"nodePath.txt" for storing path
    f = open("NodePath.txt",'w')
    
    #convert list to String
    f.writelines("%s\n" % str(move) for move in path)
    
    f.close()
    
    #NodesInfo.txt" for storing parents and children
    f2=open('NodesInfo.txt','w')
    f2.write("Node_index\tParent_Node_index\n")
    
    for row in range(len(path)):
        f2.write(str(n_index[row]))
        f2.write("\t\t\t")
        f2.write(str(p_index[row]))
        f2.write("\n")
    f2.close()
    
    #Nodes.txt" for storing all explored states/nodes
    f3=open('Nodes.txt','w')
    
    for visit in range(len(visited)):
        f3.write(str(visited[visit]))
        f3.write("\n")
    f3.close()