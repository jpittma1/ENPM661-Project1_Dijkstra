#!/usr/bin/env python3

#ENPM661 Spring 2022
#Section 0101
#Jerry Pittman, Jr. UID: 117707120
#jpittma1@umd.edu
#Project #2

from Node import *
from functions import *
from obstacles import *


'''--User input for initial and goal State--'''
Xi,Xg=GetInitialStates()

####--for testing without user input--####
# Xi = [0,0] #starts at Origin
# Xg=[380, 20] #above hexagon
# Xg=[20, 20] #above hexagon
# Xg=[200, 200] #above hexagon
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

#-------Class object and Priority Queue Initialization--------
OpenList = PriorityQueue()  
start_node = Node(Xi, None, None, 0)
OpenList.put((start_node.getCost(), start_node))

#Node Object: self, state, parent, move, cost; give each node cost of infinity

ClosedList=np.array([[Node([i,j],None, None, math.inf) for j in range(250)] for i in range(400)])

reachedGoal = False

moves_cost = {'N':1, 'NE':1.4, 'E':1, 'SE':1.4, 'S':1, 'SW':1.4, 'W':1, 'NW':1.4}

print("Initial and Goal points are valid...Generating map...")
'''**Visualization Code**
    Map Background=Black
    Start=Red
    Goal= Red
    Obstacles=Yellow
    Completed Nodes=Green
    Path=white
'''
map_size = [250, 400] 
map_y, map_x = map_size
videoname=('project2-jerry-pittman')
fourcc = cv2.VideoWriter_fourcc(*'XVID')
video = cv2.VideoWriter(str(videoname)+".avi",  fourcc, 300, (map_x, map_y))

space = np.zeros([map_size[0], map_size[1], 3], dtype=np.uint8) 
# print("space shape", space.shape)
space = updateNodesOnMap(space, Xi, [0,0,255])
space = updateNodesOnMap(space, Xg, [0,0,255])
space = addObstacles2Map(space)

cv2.imwrite('Initial_map.jpg', space)
print("Initial map created named 'Initial_map.jpg' ")

cv2.imshow('Initial_map', space)

###########Dijkstra Algorithm While Loop#############
'''Conduct Dijkstra algorithm to find path between 
initial and goal node avoiding obstacles'''

start = timeit.default_timer()
print("Commencing Dijkstra Search.......")

while not (OpenList.empty() and reachedGoal):
    
    curr_node = OpenList.get()[1]
    i, j = curr_node.getState()
    # print("current node (x,y) is: (", i, ", ",j,")")
    
    space = updateNodesOnMap(space, curr_node.getState(), [0, 255, 0])
    video.write(space)
    
    #Save Directional Moves in x and y dictionaries-----
    moves_x = {'N':i, 'NE':i+1, 'E':i+1, 'SE':i+1, 'S':i, 'SW':i-1, 'W':i-1, 'NW':i-1}
    moves_y = {'N':j+1, 'NE':j+1, 'E':j, 'SE':j-1, 'S':j-1, 'SW':j-1, 'W':j, 'NW':j+1}
    
    reachedGoal =compare2Goal(curr_node.getState(),Xg)

    if reachedGoal:
        # path = []
        print("Goal Reached!!")
        print("Total cost of path is ", curr_node.getCost())

        moves_path, path = curr_node.getFullPath()

        # print("Backtracked Moves is ", moves_path)
        # print("Backtracked Node path is ", path)
        
        for node in path:  #Make white Node pathway on map
                pos = node.getState()
                space = updateNodesOnMap(space, pos, [255, 255, 255]) #White
                cv2.imshow('Map',space)
                video.write(space)
        
        #Video ends abruptly at goal, want to have goal shown for a little longer
        for i in range(250):       
            video.write(space)
        break
    
    else:
        X_prime=possibleMoves(curr_node)
        # print("possible directions of current node are: ", X_prime)
        parent_cost=curr_node.getCost()    #current cost to come
                
        '''Iterate through all compass point directions'''
        for move in X_prime:
            child_pos = [moves_x.get(move), moves_y.get(move)]
            cost_to_come = parent_cost + moves_cost.get(move)
            
            #Verify not visited based on cost2come set to infinity
            if (ClosedList[child_pos[0], child_pos[1]].getCost() == math.inf):
                child_Node = Node(child_pos, curr_node, move, cost_to_come)
                ClosedList[child_pos[0], child_pos[1]] = child_Node
                OpenList.put((child_Node.getCost(), child_Node))
            else:
                #Check if cost is larger than c2c+local_cost
                if (cost_to_come < ClosedList[child_pos[0], child_pos[1]].getCost()): 
                    child_Node = Node(child_pos, curr_node, move, cost_to_come)
                    ClosedList[child_pos[0], child_pos[1]] = child_Node
                    OpenList.put((child_Node.getCost(), child_Node))
    
                
    if reachedGoal: break
    

stop = timeit.default_timer()
print("That algorithm took ", stop-start, " seconds")


cv2.namedWindow("map", cv2.WINDOW_NORMAL)
cv2.imshow('Final_ map', space)
cv2.imwrite('Final_map.jpg', space)
print("Final map created named 'final_map.jpg' ")

if cv2.waitKey(1) == ord('q'):
    video.release()

video.release()
cv2.destroyAllWindows()
