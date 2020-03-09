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

    secene_x = q_range[0]
    secene_y = q_range[1]
    
    n_sample = 100
    samples = [[random.uniform(secene_x[0],secene_x[1]), random.uniform(secene_y[0], secene_y[1])] for _ in range(400)]
    # samples = list(zip( list(np.linspace(x_limit[0], x_limit[1], n_sample))[1:-1], list(np.linspace(y_limit[0], y_limit[1], n_sample))[1:-1] ))
    # print('samples: ', samples)
    valid_samples = [x for x in samples if not test_in_obs(x, obstacles)]
    print("valid_samples: ", len(valid_samples))
    

    # the roadmap 
    graph = Roadmap()

    for i, sample_p in enumerate(valid_samples): #add all the valid sample vertex
        graph.addVertex(sample_p)
    
    for i_vertex in graph.getVertices():
        neighbors = k_nearest_neighbors(graph, i_vertex.getConfiguration())
        for nbor in neighbors:
            inters = interpolate(i_vertex.getConfiguration(), nbor[1].getConfiguration(), 0.1)
            if(not test_inters(inters, obstacles)):
                i_vertex.addEdge(nbor[1].id, nbor[0])
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



    return path   


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
        if( dist <= max_dist ):
            nearest_n.append((vertex, dist))

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

    #nearest_neighbors = ([x for _,x in sorted(zip(dist,graph.getVertices()))])[:K]
    nearest_neighbors = sorted(list(zip(dist, graph.getVertices())))[:K]
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

    return False 
   

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

def test_in_obs(p, obs_list):
    for obs in obs_list:
        if( obs.x_min <= p[0] <= obs.x_max and obs.y_min <= p[1] <= obs.y_max):
            return True
    return False

def test_inters(inters: list, obs_list):
    for inter_points in inters:
        if(test_in_obs(inter_points, obs_list)):
            return True
    return False

if __name__ == "__main__":
    from scene import Scene
    import tkinter as tk

    win = tk.Tk()
    Scene('prm1.csv', disk_robot, (build_roadmap, find_path), win)
    win.mainloop()
