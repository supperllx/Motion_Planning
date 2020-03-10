# prmplanner.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the authors.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)

from graph import RoadmapVertex, RoadmapEdge, Roadmap
from utils import *
from math import *
import numpy as np
import random
import time

disk_robot = True #(change this to False for the advanced extension) 


obstacles = None # the obstacles 
robot_radius = None # the radius of the robot
robot_width = None # the width of the OBB robot (advanced extension)
robot_height = None # the height of the OBB robot (advanced extension)


# ----------------------------------------
# modify the code below
# ----------------------------------------

# Construction phase: Build the roadmap
# You should incrementally sample configurations according to a strategy and add them to the roadmap, 
# select the neighbors of each sample according to a distance function and strategy, and
# attempt to connect the sample to its neighbors using a local planner, leading to corresponding edges
# See graph.py to get familiar with the Roadmap class  

def build_roadmap(q_range, robot_dim, scene_obstacles):

    global obstacles, robot_width, robot_height, robot_radius

    obstacles = scene_obstacles # setting the global obstacle variable

    x_limit = q_range[0] # the range of x-positions for the robot
    y_limit = q_range[1] # the range of y-positions for the robot
    theta_limit = q_range[2] # the range of orientations for the robot (advanced extension)

    robot_width, robot_height = robot_dim[0], robot_dim[1] # the dimensions of the robot, represented as an oriented bounding box
    
    robot_radius = max(robot_width, robot_height)/2.
    print('r: ', robot_radius)
    secene_x = q_range[0]
    secene_y = q_range[1]
    
    # samples = [[random.uniform(secene_x[0],secene_x[1]), random.uniform(secene_y[0], secene_y[1])] for _ in range(900)]
    # samples = list(zip( list(np.linspace(x_limit[0], x_limit[1], n_sample))[1:-1], list(np.linspace(y_limit[0], y_limit[1], n_sample))[1:-1] ))
    # print('samples: ', samples)

    n_sample_iters = 34
    samples = []
    for i in range(n_sample_iters):
        for j in range(n_sample_iters):
            x_low = secene_x[0] + (secene_x[1] - secene_x[0])*i/n_sample_iters
            x_high = x_low + (secene_x[1] - secene_x[0])/n_sample_iters
            y_low = secene_y[0] + (secene_y[1] - secene_y[0])*j/n_sample_iters
            y_high = y_low + (secene_y[1] - secene_y[0])/n_sample_iters
            for k in range(0, 1):
                x = random.uniform(x_low, x_high)
                y = random.uniform(y_low, y_high)
                samples.append([x,y])
    valid_samples = [x for x in samples if not test_in_obs(x, obstacles, robot_radius)]
    invalid_samples = [x for x in samples if test_in_obs(x, obstacles, robot_radius)]
    # for i in range(len(invalid_samples)):
    #     for j in range(len(invalid_samples)):
    #         mid_x = 0.5*(invalid_samples[i][0] + invalid_samples[j][0])
    #         mid_y = 0.5*(invalid_samples[i][1] + invalid_samples[j][1])
    #         if(i!=j and (not test_in_obs([mid_x, mid_y], obstacles, robot_radius))):
    #             valid_samples.append([mid_x, mid_y])
                
    print("valid_samples: ", len(valid_samples))
    

    # the roadmap 
    graph = Roadmap()

    for i, sample_p in enumerate(valid_samples): #add all the valid sample vertex
        graph.addVertex(sample_p)
    
    for i_vertex in graph.getVertices():
        # n_neighbors = nearest_neighbors(graph, i_vertex.getConfiguration(), max_dist=10)
        k_neighbors = k_nearest_neighbors(graph, i_vertex.getConfiguration(), K=6)
        # neighbors = list(set(n_neighbors).union(set(k_neighbors)))
        for nbor in k_neighbors:
            inters = interpolate(i_vertex.getConfiguration(), nbor[1].getConfiguration(), 0.5)
            if(not test_inters(inters, obstacles, robot_radius)):
                i_vertex.addEdge(nbor[1].id, nbor[0])
                nbor[1].addEdge(i_vertex.getId(), nbor[0])
                graph.addEdge(i_vertex, nbor[1], nbor[0])
    
    print('len graph: ', len(graph.vertices))
    # uncomment this to export the roadmap to a file
    graph.saveRoadmap("prm_roadmap.txt")
    return graph

# ----------------------------------------
# modify the code below
# ----------------------------------------

# Query phase: Connect start and goal to roadmap and find a path using A*
# (see utils for Value, PriorityQueue, OrderedSet classes that you can use as in project 3)
# The returned path should be a list of configurations, including the local paths along roadmap edges
# Make sure that start and goal configurations are collision-free. Otherwise return None
    
def find_path(q_start, q_goal, graph):
    path  = [] 
    
     # Use the OrderedSet for your closed list
    closed_set = OrderedSet()
    
    # Use the PriorityQueue for the open list
    open_set = PriorityQueue(order=min, f=lambda v: v.f)     

    graph.addVertex(q_start)
    graph.addVertex(q_goal)
    start_vertex = graph.getVertices()[-2]
    goal_vertex = graph.getVertices()[-1]

    start_neighbors = k_nearest_neighbors(graph, q_start, K=4)
    for nbor in start_neighbors:
        inters = interpolate(q_start, nbor[1].getConfiguration(), 0.5)
        if(not test_inters(inters, obstacles, robot_radius)):
            start_vertex.addEdge(nbor[1].id, nbor[0])
            graph.addEdge(start_vertex, nbor[1], nbor[0])

    goal_neighbors = k_nearest_neighbors(graph, q_goal, K=4)
    for nbor in goal_neighbors:
        inters = interpolate(q_goal, nbor[1].getConfiguration(), 0.5)
        if(not test_inters(inters, obstacles, robot_radius)):
            nbor[1].addEdge(goal_vertex.getId(), nbor[0])
            graph.addEdge(nbor[1], goal_vertex, nbor[0])

    open_set.put(start_vertex.getId(), Value(f=distance(q_start, q_goal), g=q_start))
    
    record = {}
    while(len(open_set) != 0):
        curr_id, curr_value = open_set.pop()
        curr_node = graph.getVertices()[curr_id]
        if curr_id != start_vertex.getId():
            record[curr_id] = curr_value.g
        closed_set.add(curr_id)
        if(curr_id == goal_vertex.getId()):
            print("Find path!")
            break
        for edge in curr_node.getEdges():
            next_id = edge.getDestination()
            next_node = graph.getVertices()[next_id]
            dist = edge.getDist()
            next_f = dist + 1*distance(next_node.getConfiguration(), goal_vertex.getConfiguration())
            if ((next_id not in closed_set)):
                if next_id not in open_set:
                    open_set.put(next_id, Value(f=next_f, g=curr_id))
                elif(open_set.has(next_id) and next_f < open_set.get(next_id).f):
                    open_set.get(next_id).f = next_f
                    open_set.get(next_id).g = curr_id

    if(curr_node.getId() != goal_vertex.getId()): 
        print('fail to find the path!')
        return path

    tag_id = goal_vertex.getId()

    while(tag_id!= start_vertex.getId()):
        father_id = record[tag_id]
        path.append(graph.getVertices()[father_id].getConfiguration())
        tag_id = father_id

    return path[::-1]


# ----------------------------------------
# below are some functions that you may want to populate/modify and use above 
# ----------------------------------------

def nearest_neighbors(graph: Roadmap, q, max_dist=10.0):
    """
        Returns all the nearest roadmap vertices for a given configuration q that lie within max_dist units
        You may also want to return the corresponding distances 
    """
    nearest_n = []

    for vertex in graph.getVertices():
        dist = distance(q, vertex.getConfiguration())
        if( 2*robot_radius< dist <= max_dist ):
            nearest_n.append((dist, vertex))

    return nearest_n


def k_nearest_neighbors(graph: Roadmap, q, K=10):
    """
        Returns the K-nearest roadmap vertices for a given configuration q. 
        You may also want to return the corresponding distances 
    """

    # dist = [0]*graph.getNrVertices()-1
    dist = []
    for i, vertex in enumerate(graph.getVertices()):
        if(q != vertex.getConfiguration()):
            dist.append(distance(q, vertex.getConfiguration()))

    # nearest_neighbors = sorted(list(zip(dist, graph.getVertices())))[:K]
    threshold = np.unique([i for i in dist if i>1*robot_radius])[K-1]
    nearest_neighbors = []
    for vertex in graph.getVertices():
        d = distance(q, vertex.getConfiguration())
        if(1*robot_radius<d<=threshold and q != vertex.getConfiguration()):
            nearest_neighbors.append((d, vertex))
    return nearest_neighbors

def distance(q1, q2): 
    """
        Returns the distance between two configurations. 
        You may want to look at the getRobotPlacement function in utils.py that returns the OBB for a given configuration  
    """

    dist = sqrt((q1[0] - q2[0])**2 + (q1[1] - q2[1])**2)

    return dist

def collision(q):
    """
        Determines whether the robot placed at configuration q will collide with the list of AABB obstacles.  
    """
    global obstacles, robot_radius

    return test_in_obs(q, obstacles, robot_radius)
   

def interpolate (q1, q2, stepsize): 
    """
        Returns an interpolated local path between two given configurations. 
        It can be used to determine whether an edge between vertices is collision-free. 
    """
    if(abs(q1[0]-q2[0]) > abs(q1[1] - q2[1])):
        size = int(abs(q1[0] - q2[0])/stepsize) + 1
        inter_samples = [[0,0]]*size
        inter_x = np.linspace(q1[0], q2[0], size)
        inter_y = np.linspace(q1[1], q2[1], size)

        for i in range(0, size):
            inter_samples[i] = [inter_x[i], inter_y[i]]
    else:
        size = int(abs(q1[1] - q2[1])/stepsize) + 1
        inter_samples = [[0,0]]*size
        inter_x = np.linspace(q1[0], q2[0], size)
        inter_y = np.linspace(q1[1], q2[1], size)

        for i in range(0, size):
            inter_samples[i] = [inter_x[i], inter_y[i]]
    return inter_samples

def test_in_obs(p, obs_list, radius):
    for obs in obs_list:
        if( (obs.x_min-radius) <= p[0] <= (obs.x_max+radius) and (obs.y_min-radius) <= p[1] <= (obs.y_max+radius)):
            return True
    return False

def test_inters(inters: list, obs_list, radius):
    for inter_points in inters:
        if(test_in_obs(inter_points, obs_list, radius)):
            return True
    return False

if __name__ == "__main__":
    from scene import Scene
    import tkinter as tk

    win = tk.Tk()
    Scene('prm1.csv', disk_robot, (build_roadmap, find_path), win)
    win.mainloop()
