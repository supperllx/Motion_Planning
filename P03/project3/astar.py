# astar.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#


# Compute the optimal path from start to goal.
# The car is moving on a 2D grid and
# its orientation can be chosen from four different directions:
forward = [[-1,  0], # 0: go north
           [ 0, -1], # 1: go west
           [ 1,  0], # 2: go south
           [ 0,  1]] # 3: go east

# The car can perform 3 actions: -1: right turn and then move forward, 0: move forward, 1: left turn and then move forward
action = [-1, 0, 1]
action_name = ['R', 'F', 'L']
cost = [1, 1, 10] # corresponding cost values

# GRID:
#     0 = navigable space
#     1 = unnavigable space 
grid = [[1, 1, 1, 0, 0, 0],
        [1, 1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0, 0],
        [1, 1, 1, 0, 1, 1],
        [1, 1, 1, 0, 1, 1]]

init = (4, 3, 0) # (grid row, grid col, orientation)
                
goal = (2, 0, 1) # (grid row, grid col, orientation)


heuristic = [[2, 3, 4, 5, 6, 7], # Manhattan distance
        [1, 2, 3, 4, 5, 6],
        [0, 1, 2, 3, 4, 5],
        [1, 2, 3, 4, 5, 6],
        [2, 3, 4, 5, 6, 7]]

from utils import (Value, OrderedSet, PriorityQueue)

"""
Two data structures are provided for your open and closed lists: 

 1. OrderedSet is an ordered collection of unique elements.
 2. PriorityQueue is a key-value container whose `pop()` method always pops out
    the element whose value has the highest priority.

 Common operations of OrderedSet, and PriorityQueue
   len(s): number of elements in the container s
   x in s: test x for membership in s
   x not in s: text x for non-membership in s
   s.clear(): clear s
   s.remove(x): remove the element x from the set s;
                nothing will be done if x is not in s

 Unique operations of OrderedSet:
   s.add(x): add the element x into the set s
   s.pop(): return and remove the LAST added element in s;

 Example:
   s = Set()
   s.add((0,1,2))    # add a triplet into the set
   s.remove((0,1,2)) # remove the element (0,1,2) from the set
   x = s.pop()

 Unique operations of PriorityQueue:
   PriorityQueue(order="min", f=lambda v: v): build up a priority queue
       using the function f to compute the priority based on the value
       of an element
   s.put(x, v): add the element x with value v into the queue
                update the value of x if x is already in the queue
   s.get(x): get the value of the element x
            raise KeyError if x is not in s
   s.pop(): return and remove the element with highest priority in s;
            raise IndexError if s is empty
            if order is "min", the element with minimum f(v) will be popped;
            if order is "max", the element with maximum f(v) will be popped.
 Example:
   s = PriorityQueue(order="min", f=lambda v: v.f)
   s.put((1,1,1), Value(f=2,g=1))
   s.put((2,2,2), Value(f=5,g=2))
   x, v = s.pop()  # the element with minimum value of v.f will be popped
"""

# ----------------------------------------
# modify the code below
# ----------------------------------------
def compute_path(grid,start,goal,cost,heuristic):
   
    # Use the OrderedSet for your closed list
    closed_set = OrderedSet()
    
    # Use thePriorityQueue for the open list
    open_set = PriorityQueue(order=min, f=lambda v: v.f)      

    # Keep track of the parent of each node. Since the car can take 4 distinct orientations, 
    # for each orientation we can store a 2D array indicating the grid cells. 
    # E.g. parent[0][2][3] will denote the parent when the car is at (2,3) facing up    
    parent = [[[' ' for row in range(len(grid[0]))] for col in range(len(grid))], 
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))],
             [[' ' for row in range(len(grid[0]))] for col in range(len(grid))]]

    # The path of the car
    path =[['-' for row in range(len(grid[0]))] for col in range(len(grid))]
    
    x = start[0]
    y = start[1]
    theta = start[2]
    h = heuristic[x][y]
    g = 0
    f = g+h
    open_set.put(start, Value(f=f,g=g))

    # your code: implement A*

    # Initially you may want to ignore theta, that is, plan in 2D.
    # To do so, set actions=forward, cost = [1, 1, 1, 1], and action_name = ['U', 'L', 'R', 'D']
    # Similarly, set parent=[[' ' for row in range(len(grid[0]))] for col in range(len(grid))]  

    #print(open_set.put((1,2,3), Value(f = 100, g = 50)))
    #print('open_set has (1,2,3): ', open_set.has((1,2,4)))
    #open_set.get((1,2,3)).f = 22
    #print("open_set.get((1,2,3)).f: ",open_set.get((1,2,3)).f)
    #print("first pop: ", open_set.pop()[1].g)
    #print("second pop: ", open_set.pop())
    path[goal[0]][goal[1]] = '*'
    
    
    while(len(open_set) != 0):
      curr_node = open_set.pop()
      if(curr_node[0] == goal): 
        print("get path!")
        closed_set.add(curr_node)
        break
      closed_set.add(curr_node)
      x_current = curr_node[0][0] #row of current node
      y_current = curr_node[0][1] #col of current node
      o_current = curr_node[0][2] #orientation of current node

      for i_act, act in enumerate(action):
        o_next = (o_current + act) % 4 #orientation of next node
        x_next = x_current + forward[o_next][0] #row of next node
        y_next = y_current + forward[o_next][1] #col of next node
        #n_act = action_name[i_act] #act of next node

        if(x_next>=0 and x_next<len(grid) and y_next>=0 and y_next<len(grid[0])): #filter the available child (map boundery and barrier)
          if(grid[x_next][y_next] == 0):
            F_cost = cost[i_act] + heuristic[x_next][y_next] # abs(x_next - goal[0]) + abs(y_next - goal[1]) #calculate the f(n) = g(n) + h(n)
            if((not open_set.has((x_next, y_next, o_next))) or (not closed_set.has((x_next, y_next, o_next)))): 
              open_set.put((x_next, y_next, o_next), Value(f = F_cost, g = i_act))
            elif(F_cost < open_set.get((x_next, y_next, o_next)).f):
              open_set.get((x_next, y_next, o_next)).f = F_cost
              open_set.get((x_next, y_next, o_next)).g = cost[i_act]   
              

    #if(curr_node[0] != goal): print("fail to find the path!")
    return path, closed_set

# def dijkstra(grid,start,goal,cost,heuristic):
#   closed_set = OrderedSet()
#   open_set = PriorityQueue(order=min, f=lambda v: v.f)

if __name__ == "__main__":
    path,closed=compute_path(grid, init, goal, cost, heuristic)

    for i in range(len(path)):
        print(path[i])

    print("\nExpanded Nodes")    
    for node, value in closed:
        print(node)

"""
To test the correctness of your A* implementation, when using cost = [1, 1, 10] your code should return 

['-', '-', '-', 'R', 'F', 'R']
['-', '-', '-', 'F', '-', 'F']
['*', 'F', 'F', 'F', 'F', 'R']
['-', '-', '-', 'F', '-', '-']
['-', '-', '-', 'F', '-', '-'] 

In this case, the elements in your closed set (i.e. the expanded nodes) are: 
(4, 3, 0)
(3, 3, 0)
(2, 3, 0)
(2, 4, 3)
(1, 3, 0)
(2, 5, 3)
(0, 3, 0)
(0, 4, 3)
(0, 5, 3)
(1, 5, 2)
(2, 5, 2)
(2, 4, 1)
(2, 3, 1)
(2, 2, 1)
(2, 1, 1)
(2, 0, 1)

"""