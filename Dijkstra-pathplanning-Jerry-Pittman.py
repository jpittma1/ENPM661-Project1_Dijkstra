#!/usr/bin/env python3

#ENPM661 Spring 2022
#Section 0101
#Jerry Pittman, Jr. UID: 117707120
#jpittma1@umd.edu
#Project #2

# import numpy as np
# import copy
# import timeit
# import queue
# from queue import PriorityQueue
# import numpy as np
# import cv2
# import scipy
# from scipy import fft, ifft
# from numpy import linalg as LA
# import matplotlib.pyplot as plt
# import sys
# import math
# from obstacles import *
# from Node import *

from Node import *
from functions import *
from obstacles import *

# Node_State_i=[]     #The state of node i  is represented by a 3 by 3 matrix
# Node_Index_i=[]         #index of node_i
# Parent_Node_Index_i=[]  #index of parent_node_i
# Node_Index_i.append(1)
# Parent_Node_Index_i.append(0)

# queue=[]                     
# Visited_Nodes=[]            #nodes that have been visited already   
# x_prime=[]                  #new nodes discovered after moving blank tile
# new_node=[]                 #after shifting node for saving into x_prime
# BackTrackedPath=[]           #to save backtracked path

# isgoal = False
success = False


# OpenList = PriorityQueue()  #nodes still to be explored
# open_set = set()
ClosedList=[]                     #nodes explored
# closed_set = set()
reachedGoal = False

'''--User input for initial and goal State--'''
# Xi,Xg=GetInitialState()

####--for testing--####
Xi = [0,0] #starts at Origin
# Xg=[380, 20] #above hexagon
Xg=[200, 210] #above hexagon
# Xg=[390,240] #behind circle
################################

print("Initial State is ", Xi)
print("Goal state is: ", Xg)

#######CHECK IF ENTERED VALUES ARE VALID###########
if isInObstacleSpace(Xi[0],Xi[1]):
    print("Initial state is in an obstacle or off the map, please provide new valid initial state")
    exit()
    
if isInObstacleSpace(Xg[0],Xg[1]):
    print("Goal state is in an obstacle or off the map, please provide new valid initial state")
    exit()


####################INITIALIZE NODEs AND MAP###############
'''Initialize tuple and store in OpenList priority queue.
       [0,1]=coordinate values (x,y) from user input
       [2]=index of node; initially set to 0
       [3]=parent node index; initially set to -1
       [4]= cost to come; initially set to 0
       [5]=total cost'''
# creating tuple with cost to come, index, parent node index=0 and coordinate values (x,y)
# Xi.append(0)            # 2 - index of node
# Xi.append(-1)           # 3 - parent node
# Xi.append(0)            # 4 - costtocome
# Xi.append(0)            # 5 - total cost
# OpenList.put((Xi[5],Xi))  #total cost, then the full node
# open_set.add(tuple(Xi[:2]))


#-------Class object for backtrack path--------
'''Rename to OpenList!!!'''
OpenList = PriorityQueue()  
start_node = Node(Xi, None, None, 0)
OpenList.put((start_node.getCost(), start_node))

#Node Object: self, state, parent, move, cost; give each node cost of infinity
'''Rename to ClosedList!!!'''
node_array=np.array([[Node([i,j],None, None, math.inf) for j in range(250)] for i in range(400)])
# init_node=(0, 0, 0, (Xi[0],Xi[1])) #Initialize Node with no parent, no move, and no cost
# OpenList.put(init_node) #Store cost and current state

moves_cost = {'N':1, 'NE':1.4, 'E':1, 'SE':1.4, 'S':1, 'SW':1.4, 'W':1, 'NW':1.4}
# map_array = np.array([[Node([i,j], None, None, math.inf) for j in range(400)] for i in range(250)])

print("Initial and Goal points are valid...Generating map...")
'''**Visualization Code**
    Map Background=Black
    Start=Red
    Goal= Red
    Obstacles=Yellow
    Completed Nodes=Green
    Path=white
    
    fliptransform X & Y because of openCv has origin at top left

'''
map_size = [250, 400] 
map_y, map_x = map_size
videoname=('project1-jerry-pittman')
fourcc = cv2.VideoWriter_fourcc(*'XVID')
video = cv2.VideoWriter(str(videoname)+".avi",  fourcc, 300, (map_x, map_y))

# space = np.zeros((map_x, map_y, 3), np.uint8)
space = np.zeros([map_size[0], map_size[1], 3], dtype=np.uint8) 

# print("space shape", space.shape)
space = updateNodesOnMap(space, Xi, [255,255,0])
space = updateNodesOnMap(space, Xg, [255,255,0])
space = addObstacles2Map(space)

cv2.imwrite('Initial_map.jpg', space)
print("Initial map created named 'Initial_map.jpg' ")

#Flip and set origin to bottom left
# space = np.zeros((map_x, map_y,3), np.uint8) 
# space[:,:,0] = 0
# space[:,:,1] = 0
# space[:,:,2] = map

# cv2.imwrite('space.jpg', space)

# cv2.imwrite('map.jpg', space)
# cv2.imshow('frame', space)

###########Dijkstra Algorithm While Loop#############
'''Conduct Dijkstra algorithm to find path between 
initial and goal node avoiding obstacles'''

start = timeit.default_timer()
print("Commencing Dijkstra Search.......")
# moves = ['N','NE','E','SE','S','SW','W','NW']

count=0
while not (OpenList.empty() and reachedGoal):
   
    # init_node=OpenList.get()
    # print("init_node is ", init_node)
    # current_node = init_node[1][:]
    # print("current_node is ", current_node)
    
    curr_node = OpenList.get()[1]
    i, j = curr_node.getState()
    # print("current node (x,y) is: (", i, ", ",j,")")
    
    space = updateNodesOnMap(space, curr_node.getState(), [0, 255, 0])
    video.write(space)
    
    # open_set.remove(tuple(current_node[:2]))
    # ClosedList.append(current_node)
    # closed_set.add(tuple(current_node[:2]))
    
    #Moves in x and y dictionaries
    moves_x = {'N':i, 'NE':i+1, 'E':i+1, 'SE':i+1, 'S':i, 'SW':i-1, 'W':i-1, 'NW':i-1}
    moves_y = {'N':j+1, 'NE':j+1, 'E':j, 'SE':j-1, 'S':j-1, 'SW':j-1, 'W':j, 'NW':j+1}
    
    reachedGoal =compare2Goal(curr_node.getState(),Xg)
    # if goal_reached or count == 100000: #400*250=100000
    if reachedGoal:
        # path = []
        print("Goal Reached!!")
        print("Total cost of path is ", curr_node.getCost())
        # print("Total cost of path is ", init_node[0])
        moves_path, path = curr_node.getFullPath()
        # goal_reached = True
        
        # parent_index = current_node[3]      #save the parent's index
        # while (parent_index != -1):          #is only set to -1 initially
        #     node = ClosedList[parent_index]
        #     path.append((node[0],node[1]))  #save x and y to path
        #     parent_index = node[3]          #update parent index
            
        # path = list(reversed(path))         #create backtrackpath
        # path.append((Xg[0],Xg[1]))          #add goal node to path
        # print("Backtracked Moves is ", moves_path)
        # print("Backtracked Node path is ", path)
        
        for node in path:  #make white pathway on map
                pos = node.getState()
                space = updateNodesOnMap(space, pos, [255, 255, 255]) #White
                cv2.imshow('Map',space)
                video.write(space)
        
        break
    
    else:
        X_prime=possibleMoves(curr_node)
        # print("possible directions of current node are: ", X_prime)
        parent_cost=curr_node.getCost()    #current cost to come
                
        
        '''nodes to OpenList; node_array to ClosedList'''
        '''Iterate through all compass point directions'''
        for move in X_prime:
            child_pos = [moves_x.get(move), moves_y.get(move)]
            cost_to_come = parent_cost + moves_cost.get(move)
            
            #Verify not visited based on cost2come set to infinity
            if (node_array[child_pos[0], child_pos[1]].getCost() == math.inf):
                child_Node = Node(child_pos, curr_node, move, cost_to_come)
                node_array[child_pos[0], child_pos[1]] = child_Node
                OpenList.put((child_Node.getCost(), child_Node))
            else:
                #Check if cost is larger than c2c+local_cost
                if (cost_to_come < node_array[child_pos[0], child_pos[1]].getCost()): 
                    child_Node = Node(child_pos, curr_node, move, cost_to_come)
                    node_array[child_pos[0], child_pos[1]] = child_Node
                    OpenList.put((child_Node.getCost(), child_Node))
    
    
    # tmp=possibleMoves(current_node)
        # print("tmp", tmp)
        # parent_cost=init_node[4]    #current cost to come
        
        '''Iterate through all compass point directions'''
        # for direction in moves:
        #     X_prime, cost = ActionMove(current_node, direction)
        #     print("X_prime[:2] is", X_prime[:2])
        # if not ((tuple(X_prime[:2]) in closed_set) and isInObstacleSpace(X_prime[0],X_prime[1])):
        #         if not (tuple(X_prime[:2]) in open_set):
        #             X_prime[3] = current_node[2]        #update index
        #             X_prime[4] = current_node[4] + cost #update cost to come
        #             X_prime[5] = X_prime[4]            #update total cost
        #             OpenList.put((X_prime[5],X_prime))  #total cost, nodes
        #             open_set.add(tuple(X_prime[:2]))    #update open_set
        #         else:   #update Nodes 
        #             if(X_prime[5] > X_prime[4] + cost): #if total cost is larger than c2c+cost
        #                 X_prime[3] = current_node[2]
        #                 X_prime[4] = current_node[4] + cost
        #                 X_prime[5] = X_prime[4]
        #             # space = updateNodesOnMap(space, current_node, [0, 255, 0]) #green  
    # space = updateNodesOnMap(space, current_node, [0, 255, 0])               
                
    if reachedGoal: break
    
    count+=1
    
    # i,j = current_node[3]
    # print("current_node location is ", current_node[3])

    # ClosedList.append(current_node)

    # space = updateNodesOnMap(space, current_node, [0, 255, 0])

    #    cv2.imshow('frame',space)
    # # video.write(space)
    #UPDATE MAP BASED ON NEW POSITION

    #    moves_x={'N':i, 'NE':i+1, 'E':i+1, 'SE':i+1, 'S':i, 'SW':i-1, 'W':i-1, 'NW':i-1}
    #    moves_y={'N':j+1, 'NE':j+1, 'E':j, 'SE':j-1, 'S':j-1, 'SW':j-1, 'W':j, 'NW':j+1}

    # print("Queue start is ", Node_State_i)
    # print("Goal state is ", Goal_State)

    #'''Verify if current Node is Goal Node returns: 
    #True if are same/reached; 
    #False if are different/not reached'''
    
    # reachedGoal=compare2Goal(current_node[3],Xg)
   
    # if reachedGoal==True:
    #     print("Goal Reached!!")
    #     # print("Total cost of path: ", total_cost)
    #     ###cost2come_total
    #     '''Back track path here...'''
        
    #     '''Make plot/video'''
    #     results=current_node[3]
    #     break
    
    # else:
        
    #     x_prime=determinePossibleMoves(current_node)
        
        # if x_prime not in ClosedList and x_prime is not ####IN OBSTACLE SPACE####:
        #     if x_prime not in OpenList or cost(x_prime) == math.inf:
        #         print("Not in OpenList or x_prinme==math.inf")
        # cost2come_total=

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
stop = timeit.default_timer()
print("That algorithm took ", stop-start, " seconds")

'''Back tracked Path function'''
# # print("length of pathBackwards", len(BackTrackedPath))
# pathway=generate_path(Initial_State, results, BackTrackedPath)
# print("Pathway is ", pathway)

# print("Making .txt files..")
# makeFiles(Visited_Nodes, results, pathway, Parent_Node_Index_i, Node_Index_i)
# print("Program complete.")

# cv2.namedWindow("map", cv2.WINDOW_NORMAL)
cv2.imshow('map', space)
cv2.imwrite('Final_map.jpg', space)

if cv2.waitKey(1) == ord('q'):
    video.release()
# cv2.waitKey(0)

video.release()
cv2.destroyAllWindows()