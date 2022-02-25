#!/usr/bin/env python3

#ENPM673 Spring 2022
#Section 0101
#Jerry Pittman, Jr. UID: 117707120
#jpittma1@umd.edu
#Project #2 Functions

import numpy as np
import cv2
import scipy
from scipy import fft, ifft
from numpy import linalg as LA
import matplotlib.pyplot as plt
import sys
import math
from obstacles import *

'''OpenCV/ Visualization Functions'''
def updateNodesOnMap(map, node_state, color):
    x,y, _ = map.shape
    trans_y=node_state[0]
    trans_x=x-node_state[1] - 1
    map[trans_x,trans_y, :] = color
    
    return map

def addObstacles2Map(map):
    
    #circle
    # cv2.circle(map, (circle_offset_x, circle_offset_y),circle_radius, (0,255,255),-1)
    cv2.circle(map, (300, 185),45, (0,255,255),-1)
    # map=cv2.circle(map, [300,185],circle_radius, (250,0,0),3)
    # cv2.circle(map, [300,185],circle_radius, (255,0,0),3)
    
    # cv2.imwrite('map.jpg', map)
    # cv2.imshow(map)
    
    #boomerang
    # cv2.polylines(map,[boomerang_pts],True,(0,255,255))
    cv2.fillConvexPoly(map,boomerang_pts,(0,255,255))
    
    #hexagon
    # cv2.polylines(map,[hexagon_pts],True,(0,255,255))
    cv2.fillConvexPoly(map,hexagon_pts,(0,255,255))
    
    # result = cv2.pointPolygonTest(contour, (x,y), False) 
    # positive (inside), negative (outside), or zero (on an edge) value,
    # In the function, the third argument is measureDist. If it is True, it finds the shortest distance between a point in the image and a contour. If False, it finds whether the point is inside, 
    # outside, or on the contour. Since we don't want to find the distance, we set the measureDist argument to False


    # black_frame = np.zeros_like(your_frame).astype(np.uint8)
    # cv2.fillPoly(black_frame , [hull], (255, 255, 255))
    
    return map
#Return 1 if within an obstacle or outside of map
def isInObstacleSpace(x,y):
    x_max=400
    y_max=250
    '''positive (inside), negative (outside), or zero (on an edge) value,
    In the function, the third argument is measureDist. If it is True, it finds the
    shortest distance between a point in the image and a contour. If False, it finds
    whether the point is inside, outside, or on the contour. Since we don't want to
    find the distance, we set the measureDist argument to False'''
    
    #Check if within Map
    if x>(x_max-1) or x<0 or y<0 or y>(y_max-1):
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
    in_boomerang=cv2.pointPolygonTest(boomerang_pts, (x,y), False)
    if in_boomerang>0:
        return 1
    
    return 0

def determinePossibleMoves(node):
    i, j = node[3]
    possibleMoves=[]
    
    possibleMoves.append(ActionMoveUp(i,j))
    ActionMoveUpRight(i,j)
    ActionMoveRight(i,j)
    ActionMoveDownRight(i,j)
    ActionMoveDown(i,j)
    ActionMoveDownLeft(i,j)
    ActionMoveLeft(i,j)
    ActionMoveUpLeft(i,j)
    
    
    moves = ['N','NE', 'E', 'SE', 'S', 'SW','W', 'NW']
    final_moves = ['N','NE', 'E', 'SE', 'S', 'SW','W', 'NW']
    move_i = [i, i+1, i+1, i+1, i, i-1, i-1, i-1]
    move_j = [j+1, j+1, j, j-1, j-1, j-1, j, j+1]
    
    for move in range(len(moves)):
        
        ActionMoveDown
        ActionMoveDownLeft
        ActionMoveDownRight
        ActionMoveUp
        ActionMoveUpRight
        ActionMoveUpLeft
        ActionMoveLeft
        ActionMoveRight
        
        if (isInObstacleSpace(move_i[move], move_j[move]) or current_node.getParentState() == [move_i[move], move_j[move]]):
            final_moves.remove(moves[move])
    # print(final_moves)
    return possibleMoves

'''-----8 Subfunctions for actions-----
Action sets= {(1,0), (-1,0), (0,1), 
(0,-1), (1,1), (-1,1),(1,-1),(-1,-1)}
moves_x={'N':i, 'NE':i+1, 'E':i+1, 'SE':i+1, 
'S':i, 'SW':i-1, 'W':i-1, 'NW':i-1}
   moves_y={'N':j+1, 'NE':j+1, 'E':j, 'SE':j-1, 
   'S':j-1, 'SW':j-1, 'W':j, 'NW':j+1}'''
def ActionMoveLeft(CurrentNode):
    NewNode = CurrentNode.copy()
    position = NewNode.index(0)
    
    tmp = NewNode[position]
    NewNode[position] = NewNode[position - 1]
    NewNode[position - 1] = tmp
    return NewNode

def ActionMoveRight(CurrentNode):
    NewNode = CurrentNode.copy()
    position = NewNode.index(0)

    tmp = NewNode[position]
    NewNode[position] = NewNode[position + 1]
    NewNode[position + 1] = tmp
    return NewNode

def ActionMoveUp(CurrentNode):
    NewNode = CurrentNode.copy()
    position = NewNode.index(0)

    tmp = NewNode[position]
    NewNode[position] = NewNode[position - 3]
    NewNode[position - 3] = tmp
    return NewNode

def ActionMoveDown(CurrentNode):
    NewNode = CurrentNode.copy()
    position = NewNode.index(0)

    tmp = NewNode[position]
    NewNode[position] = NewNode[position + 3]
    NewNode[position + 3] = tmp
    return NewNode

def ActionMoveUpRight(CurrentNode):
    NewNode = CurrentNode.copy()
    position = NewNode.index(0)

    tmp = NewNode[position]
    NewNode[position] = NewNode[position + 3]
    NewNode[position + 3] = tmp
    return NewNode

def ActionMoveDownRight(CurrentNode):
    NewNode = CurrentNode.copy()
    position = NewNode.index(0)

    tmp = NewNode[position]
    NewNode[position] = NewNode[position + 3]
    NewNode[position + 3] = tmp
    return NewNode

def ActionMoveDownLeft(CurrentNode):
    NewNode = CurrentNode.copy()
    position = NewNode.index(0)

    tmp = NewNode[position]
    NewNode[position] = NewNode[position + 3]
    NewNode[position + 3] = tmp
    return NewNode

def ActionMoveUpLeft(CurrentNode):
    NewNode = CurrentNode.copy()
    position = NewNode.index(0)

    tmp = NewNode[position]
    NewNode[position] = NewNode[position + 3]
    NewNode[position + 3] = tmp
    return NewNode


'''Check for Goal Node Function'''
#################


##################

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
    
def GetInitialState():
    print("Enter initial node, separated by spaces: ")
    initial=[int(x) for x in input().split()]
    print("Enter goal node, separated by spaces: ")
    final=[int(x) for x in input().split()]
    return initial, final

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