#!/usr/bin/env python3

#ENPM661 Spring 2022
#Section 0101
#Jerry Pittman, Jr. UID: 117707120
#jpittma1@umd.edu
#Project #2

import numpy as np
import copy
import timeit
import queue
from queue import PriorityQueue
import cv2
from Node import *
from functions import *
from obstacles import *




# Node_State_i=[]     #The state of node i  is represented by a 3 by 3 matrix
Node_Index_i=[]         #index of node_i
Parent_Node_Index_i=[]  #index of parent_node_i
Node_Index_i.append(1)
Parent_Node_Index_i.append(0)

# queue=[]                     
Visited_Nodes=[]            #nodes that have been visited already   
x_prime=[]                  #new nodes discovered after moving blank tile
new_node=[]                 #after shifting node for saving into x_prime
BackTrackedPath=[]           #to save backtracked path


OpenList = PriorityQueue()  #nodes still to be explored
ClosedList=[]                     #nodes explored
goal_reached=False

'''--User input for initial and goal State---'''
# Xi,Xg=GetInitialState()

#--for testing--
Xi = [0,0] #starts at Origin
Xg=[100, 200] #above hexagon
# Xg=[200, 220] #above hexagon
# Xg=[390,240] #behind circle
print("Initial State is ", Xi)
print("Goal state is: ", Xg)

##################
#VERIFY NOT IN OBSTACLE SPACE

#####################

'''Initialize tuple and store in OpenList priority queue.
       [0]=cost to come; initially set to 0
       [1]=index; initially set to 0
       [2]=parent node index; initially set to 0
       [3]= coordinate values (x,y) Initially set to Xi'''
# creating tuple with cost to come, index, parent node index=0 and coordinate values (x,y)
init_node=(0, 0, 0, (Xi[0],Xi[1])) #Initialize Node with no parent, no move, and no cost
OpenList.put(init_node) #Store cost and current state

moves_cost = {'N':1, 'NE':1.4, 'E':1, 'SE':1.4, 'S':1, 'SW':1.4, 'W':1, 'NW':1.4}
# map_array = np.array([[Node([i,j], None, None, math.inf) for j in range(400)] for i in range(250)])


'''**Visualization Code**
    Map Background=Black
    Start=Red
    Goal= Red
    Obstacles=Yellow
    Completed Nodes=Green

'''
map_size = [400, 250]
map_y, map_x = map_size
# video = cv2.VideoWriter('project1-jerry-pittman',  
#                          cv2.VideoWriter_fourcc(*'MJPG'), 
#                          300, (map_x, map_y))

# space = np.zeros((map_x, map_y, 3), np.uint8)
# # print("space shape", space.shape)
# space = updateNodesOnMap(space, Xi, [0,0,255])
# space = updateNodesOnMap(space, Xg, [0,0,255])
# space = addObstacles2Map(space)

##################
#VERIFY NOT IN OBSTACLE SPACE

#####################

# cv2.imwrite('map.jpg', space)
# cv2.imshow('frame', space)
'''Conduct Dijkstra algorithm to find path between initial and goal node avoiding obstacles'''

start = timeit.default_timer()
print("Commencing Dijkstra Search.......")
while (not OpenList.empty() and goal_reached==False):
   
   current_node=OpenList.get()
   print("current_node is ", current_node)
   i,j = current_node[3]
   print("current_node location is ", current_node[3])
   
   ClosedList.append(current_node)
   
#    space = updateNodesOnMap(space, current_node.getState(), [0, 255, 0])
   
#    cv2.imshow('frame',space)
   # # video.write(space)
   #UPDATE MAP BASED ON NEW POSITION
   
#    moves_x={'N':i, 'NE':i+1, 'E':i+1, 'SE':i+1, 'S':i, 'SW':i-1, 'W':i-1, 'NW':i-1}
#    moves_y={'N':j+1, 'NE':j+1, 'E':j, 'SE':j-1, 'S':j-1, 'SW':j-1, 'W':j, 'NW':j+1}

    # print("Queue start is ", Node_State_i)
    # print("Goal state is ", Goal_State)
    '''#####Convert to a function...#####'''
    if np.array_equal(current_node[3], Xg):
        print("Goal Reached!!")
        # print("Total cost of path: ", total_cost)
        '''Back track path here...'''
        '''Make plot/video'''
        results=current_node[3]
        break
    
    else:
        
        x_prime=determinePossibleMoves(current_node)
        
        if x_prime not in ClosedList and x_prime is not ####IN OBSTACLE SPACE####:
            if x_prime not in OpenList or x_prime == math.inf:
        
        cost2come_total=

    # Finds Blank tile location then perform BFS based on 
    #     possible moves in that location location
    # x_prime=BFSsearch(current_node)
    # print("BFS Search found ", len(x_prime), "nodes to test")
    # print("x_prime is ", x_prime)
    
    '''For generating Backtracked path'''
    # for node in x_prime:
    #     tmp=[]
    #     # print("node", node)
    #     # print("x+prime[node]", x_prime[node])
    #     tmp.append(node)            #child
    #     tmp.append(current_node)     #parent
    #     BackTrackedPath.append(tmp)
        # print("backTrackedPath", BackTrackedPath)
    
    # #verify if new nodes discovered have been explored
    # for branch in x_prime:
    #     # print("Branch is ", branch)
    #     # print("Visited_Nodes are ", Visited_Nodes)
    #     # print(branch not in Visited_Nodes)
    #     if branch not in Visited_Nodes:
    #         Visited_Nodes.append(branch)
    #         queue.append(branch)
    #         # print("Branch is ", branch)

    # print("Visited nodes is ", len(Visited_Nodes), "long, and queue is ", len(queue))

    # x_prime.clear()
    
# print("BFS search Complete...Generating Path...")
# stop = timeit.default_timer()
# print("That search took ", stop-start, " seconds")

'''Back tracked Path function'''
# # print("length of pathBackwards", len(BackTrackedPath))
# pathway=generate_path(Initial_State, results, BackTrackedPath)
# print("Pathway is ", pathway)

# print("Making .txt files..")
# makeFiles(Visited_Nodes, results, pathway, Parent_Node_Index_i, Node_Index_i)
# print("Program complete.")
cv2.waitKey(0) 

cv2.destroyAllWindows()