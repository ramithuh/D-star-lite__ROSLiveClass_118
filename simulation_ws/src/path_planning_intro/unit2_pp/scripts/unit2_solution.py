#! /usr/bin/env python

"""
DStarLite's algorithm path planning exercise solution
Author: Roberto Zegers R.
Copyright: Copyright (c) 2020, Roberto Zegers R.
License: BSD-3-Clause
Date: Nov 30, 2020
Usage: roslaunch unit2_pp unit2_solution.launch
"""

import rospy
import heapq
import math

class Vertex:
    '''
        Defines a vertex by its grid position (x,y) and stores its key 
        K - Key has two values [k1,k2]
    '''
    def __init__(self, x, y, k1, k2):
        self.x  = int(x)
        self.y  = int(y)
        self.k1 = k1
        self.k2 = k2
        self.k  = [k1,k2]
        self.info = [k1,k2,[x,y]]

km = 0

#define width 50
#define height 50

width  = 74
height = 74 #will be overwitten

rhs  = []
GRID = []
g    = []

priorityQueue = [] #create empty list

s_start = Vertex(0,0,0,0)
s_goal = Vertex(0,0,0,0)

def isVertexEqual(v1,v2):
    if(v1.x == v2.x and v1.y == v2.y):
        return 1
    return 0

def h(s1,s2):
    #heuristic function
    return math.sqrt((s1.x-s2.x)**2 + (s1.y-s2.y)**2)

def CalculateKey(s):
    k1 = min(g[s.x][s.y],rhs[s.x][s.y]) + h(s_start,s) + km
    k2 = min(g[s.x][s.y],rhs[s.x][s.y])

    #s.k1 = k1
    #s.k2 = k2
    updatedVertex = Vertex(s.x,s.y,k1,k2)
    return updatedVertex

def initialize():
    global priorityQueue
    global s_goal

    priorityQueue = []
    heapq.heapify(priorityQueue) #heapify

    global km 
    km = 0

    for i in range(0,width):
        for j in range(0,height):
            rhs[i][j] = float('inf')
            g[i][j]   = float('inf')
    
    rhs[s_goal.x][s_goal.y] = 0
    s_goal = CalculateKey(s_goal)
    heapq.heappush(priorityQueue,s_goal.info)

def cg_cost(a, b):
    global GRID

    if(b.x < 0 or b.x > width - 1 or b.y < 0 or b.y > height - 1): #checks if intended place is outof bounds
        return float('inf')


    blocked = GRID[a.x][a.y] + GRID[b.x][b.y]

    if(blocked > 0):
        return float('inf')
    else:
        cost =  math.sqrt( (a.x - b.x)**2 + (a.y - b.y)**2) + g[b.x][b.y]

        if(cost >= float('inf')):
            return float('inf')
        else:
            return cost

def removeIfExist(u):
    '''
        Checks if vertex u exists in Priority Queue, and removes it
    '''
    global priorityQueue
    for item in priorityQueue:
        if([u.x,u.y] == item[2]):
            priorityQueue.remove(item)
            priorityQueue.sort()
            return True
    
    return False

def UpdateVertex(u):
    global priorityQueue
    global s_goal
    global width
    global height
    #check if out of bounds
    if(u.x < 0 or u.x >= width or u.y < 0 or u.y >= height):
        return
    if(not(isVertexEqual(u,s_goal))):
        c1 = float('inf')
        c2 = float('inf')
        c3 = float('inf')
        c4 = float('inf')
        c5 = float('inf')
        c6 = float('inf')
        c7 = float('inf')
        c8 = float('inf')

        if(u.y+1 > height):c1 = float('inf')
        else: c1 = cg_cost(u,Vertex(u.x,u.y+1,0,0))

        if(u.x+1 > width):c2 = float('inf')
        else: c2 = cg_cost(u,Vertex(u.x+1,u.y,0,0))

        if(u.y-1 < 0):c3 = float('inf')
        else: c3 = cg_cost(u,Vertex(u.x,u.y-1,0,0))

        if(u.x-1 < 0):c4 = float('inf')
        else: c4 = cg_cost(u,Vertex(u.x-1,u.y,0,0))

        if(u.x-1 < 0 or u.y - 1 < 0):c5 = float('inf')
        else: c5 = cg_cost(u,Vertex(u.x-1,u.y-1,0,0))

        if(u.x-1 < 0 or u.y + 1 > height): c6 = float('inf')
        else: c6 = cg_cost(u,Vertex(u.x-1,u.y+1,0,0))

        if(u.x + 1 > width or u.y - 1 < 0): c7 = float('inf')
        else: c7 = cg_cost(u,Vertex(u.x+1,u.y-1,0,0))

        if(u.x + 1 > width or u.y + 1 > height): c8 = float('inf')
        else: c8 = cg_cost(u,Vertex(u.x+1,u.y+1,0,0))

        rhs[u.x][u.y] = min(min(min(c3,c4),min(c1,c2)),min(min(c7,c8),min(c5,c6)))
    
    removeIfExist(u) #if U in PriorityQueue, remove it
    
    if(rhs[u.x][u.y]!=g[u.x][u.y]):
        u = CalculateKey(u)
        heapq.heappush(priorityQueue,u.info)

def isCostLower(b, a):  
    if(b.k1 < a.k1):
        return 1
    elif(a.k1 == b.k1):
        if(b.k2 < a.k2):
            return 1
        else:
            return 0;
    else:
        return 0;

def TopKey():
    '''
        Returns top vertex as the vertex class type
    '''
    global priorityQueue
    if(len(priorityQueue)==0):
        return Vertex(0,0,float('inf'),float('inf'));
    top_i = priorityQueue[0]

    top_vertex = Vertex(top_i[2][0],top_i[2][1],top_i[0],top_i[1])
    return top_vertex

def ComputeShortestPath():
    global priorityQueue
    global s_start


    while(isCostLower(TopKey(),CalculateKey(s_start)) or 
            rhs[s_start.x][s_start.y] != g[s_start.x][s_start.y]):

        k_old = TopKey()

        cell_index = getIndex(k_old.x,k_old.y)
        rospy.loginfo('size = '+ str(len(priorityQueue))+ " at - " + str(k_old.x)+','+str(k_old.y))

        heapq.heappop(priorityQueue)
        u     = k_old;

        #rospy.loginfo('rhs,g ' + str(rhs[s_start.x][s_start.y])+','+str(g[s_start.x][s_start.y]))

        if(k_old.k1 == float('inf') ): #break if path doesn't exist
            return;

        if(isCostLower(k_old,CalculateKey(u))):
            u = CalculateKey(u);
            heapq.heappush(priorityQueue,u.info)

        elif(g[u.x][u.y] > rhs[u.x][u.y]):
            g[u.x][u.y] = rhs[u.x][u.y];

            UpdateVertex(Vertex(u.x   ,u.y+1,0,0));
            UpdateVertex(Vertex(u.x+1 ,u.y  ,0,0));
            UpdateVertex(Vertex(u.x   ,u.y-1,0,0));
            UpdateVertex(Vertex(u.x-1 ,u.y  ,0,0));

            UpdateVertex(Vertex(u.x -1  ,u.y-1,0,0));
            UpdateVertex(Vertex(u.x -1  ,u.y+1,0,0));
            UpdateVertex(Vertex(u.x +1  ,u.y-1,0,0));
            UpdateVertex(Vertex(u.x +1  ,u.y+1,0,0));
        else:
            g[u.x][u.y] = float('inf');

            UpdateVertex(Vertex(u.x   ,u.y  ,0,0));

            UpdateVertex(Vertex(u.x   ,u.y+1,0,0));
            UpdateVertex(Vertex(u.x+1 ,u.y  ,0,0));
            UpdateVertex(Vertex(u.x   ,u.y-1,0,0));
            UpdateVertex(Vertex(u.x-1 ,u.y  ,0,0));

            UpdateVertex(Vertex(u.x -1  ,u.y-1,0,0));
            UpdateVertex(Vertex(u.x -1  ,u.y+1,0,0));
            UpdateVertex(Vertex(u.x +1  ,u.y-1,0,0));
            UpdateVertex(Vertex(u.x +1  ,u.y+1,0,0));
'''
0000
0000
0000
00X0

(3,2)

11

(11%4, 11/4)
'''


def getIndex(i,j):
    return j*width + i;

def convertToCellIndex(x,y,resolution):
    cellIndex = 0
    newX = x/resolution
    newY = y/resolution

    cellIndex = getIndex(newY,newX)
    return int(cellIndex)

def getRow(index):
    return int(index % width)
    
def getCol(index):
    return index//width

def outofbounds(v):
    if(v.x < 0 or v.x >= height or v.y < 0 or v.y >=width ):
        return 1
    else:
        return 0

def onestep():
    
    '''
       Selecting the next lowest cost node and assigns it to s_start
    '''

    moves = [[-1,0],[0,-1],[1,0],[0,1],[-1,-1],[-1,1],[1,-1],[1,1]]

    global s_start
    global s_goal

    rospy.loginfo("currently -> "+str(s_start.x) + "," + str(s_start.y) + ' => going to '+str(s_goal.x) + "," + str(s_goal.y))



    if(s_start.x == s_goal.x and s_start.y == s_goal.y):
        return 0

    arr = []

    for i in range(0,len(moves)):
        arr.append(cg_cost(s_start, Vertex(s_start.x + moves[i][0] ,s_start.y + moves[i][1],0,0)))
 
    min_index = arr.index(min(arr))

    s_start.x = s_start.x + moves[min_index][0]
    s_start.y = s_start.y + moves[min_index][1]

    if(not(outofbounds(s_start))):       
        return 1
    else:
        rospy.loginfo("went out of bounds")
        return 0


def d_star_lite(start_index, goal_index, width_, height_, costmap, resolution, origin, grid_viz):
    ''' 
    Performs DStarLite's shortes path algorithm search on a costmap with a given start and goal node
    '''
    global s_start
    global s_goal
    global width
    global height
    global rhs
    global GRID
    global g

    width = width_
    height = height_
    
    rhs  = [[0 for x in range(width)] for y in range(height)] 
    GRID = [[0 for x in range(width)] for y in range(height)] 
    g    = [[0 for x in range(width)] for y in range(height)] 


    rospy.loginfo('DStarLite: Starting... ')
    rospy.loginfo('width ' + str(width))
    rospy.loginfo('height ' + str(height))


    s_start = Vertex(getRow(start_index),getCol(start_index),0,0)
    s_goal  = Vertex(getRow(goal_index),getCol(goal_index),0,0)

    rospy.loginfo("start = " + str(s_start.x) +','+ str(s_start.y))
    rospy.loginfo("goal  = " + str(s_goal.x)  +','+ str(s_goal.y))

    initialize()

    #rospy.loginfo(costmap)

    x = 0
    for row in range(0,height):
        for col in range(0,width):
            if(costmap[x] > 1):
                GRID[col][row] = 1
            else:
                GRID[col][row] = 0
            x+=1

    for i in range(0,height):
        for j in range(0,width):
            if(i == s_goal.x and j == s_goal.y):
                print("X",end="")
            elif(i == s_start.x and j == s_start.y):
                print("S",end="")
            else:
                print(GRID[i][j],end="")
        print()

    '''
    output = []
    for i in range(0,width):
        output.append([])
        for j in range(0,height):
            output[i].append(str(GRID[i][j]))

    for row in output:
        rospy.loginfo(row)
    '''

    rospy.loginfo('DStarLite: Done with initialization')

    ComputeShortestPath()

    bestPath = []

    #if(g[s_start.x][s_start.y]!=float('inf')):
    rospy.loginfo("DStarLite: Going to construct Path")
    while(onestep()):
        bestPath.append(getIndex(s_start.x,s_start.y))

    rospy.loginfo('DStarLite: Done reconstructing path')

    return bestPath