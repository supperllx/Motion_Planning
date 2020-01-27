# agent.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the author.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#


import numpy as np
import random
from math import *

class Agent(object):

    def __init__(self, csvParameters, dhor = 10, goalRadiusSq=1):
        """ 
            Takes an input line from the csv file,  
            and initializes the agent
        """
        self.id = int(csvParameters[0]) # the id of the agent
        self.gid = int(csvParameters[1]) # the group id of the agent
        self.pos = np.array([float(csvParameters[2]), float(csvParameters[3])]) # the position of the agent 
        self.vel = np.zeros(2) # the velocity of the agent
        self.goal = np.array([float(csvParameters[4]), float(csvParameters[5])]) # the goal of the agent
        self.prefspeed = float(csvParameters[6]) # the preferred speed of the agent
        self.gvel = self.goal-self.pos # the goal velocity of the agent
        self.gvel = self.gvel/(sqrt(self.gvel.dot(self.gvel )))*self.prefspeed       
        self.maxspeed = float(csvParameters[7]) # the maximum sped of the agent
        self.radius = float(csvParameters[8]) # the radius of the agent
        self.goalRadiusSq =goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.dhor = dhor # the sensing radius
        self.vnew = np.zeros(2) # the new velocity of the agent     
    
    
    def computeNewVelocity(self, neighbors=[]):
        """ 
            Your code to compute the new velocity of the agent. 
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors.
            The code should set the vnew of the agent to be one of the sampled admissible one. Please do not directly set here the agent's velocity.   
        """       
        if not self.atGoal:
            k = 4 # number of the nearest agent who are chosen
            N = 200 # number of candidate vnew we will create

            valid_neighbors = self.in_range(neighbors, self.dhor)
            n_nearest = min(k, len(valid_neighbors))
            dist = []
            for i in valid_neighbors:
                dist.append(self.get_distance(i))

            nearest_neighbors = ([x for _,x in sorted(zip(dist,valid_neighbors))])[:n_nearest]
            
            if(len(nearest_neighbors)>0):
                vnew_list = self.sample_vnew(N)
                min_cost = float('inf')
                for cand_vnew in vnew_list:
                    tc = self.get_tc(cand_vnew, nearest_neighbors)
                    if(tc == 0):
                        break
                    else:
                        cost = self.get_cost(cand_vnew, tc)
                        if(cost < min_cost):
                            self.vnew = cand_vnew
                            min_cost = cost
                #self.vnew = best_vnew

            else:
                self.vnew = self.gvel  

    def update(self, dt):
        """ 
            Code to update the velocity and position of the agent  
            as well as determine the new goal velocity 
        """
        if not self.atGoal:
            #print("------------")
            #print("ID: ",self.id, "vnew: ", self.vnew, "gvel: ", self.gvel)
            self.vel[:] = self.vnew[:]
            self.pos += self.vel*dt   #update the position
        
            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                self.atGoal = True  # goal has been reached
            else: 
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed  

    def in_range(self, neighbors,dh):
        valid_neighbors = []
        for i in neighbors:
            distance  =  sqrt((self.pos[0] - i.pos[0])**2 + (self.pos[1] - i.pos[1])**2)
            if(i.id != self.id and distance <= dh ): valid_neighbors.append(i)
        return valid_neighbors

    def get_tau(self, v0, target):
        r = self.radius+target.radius
        x = self.pos - target.pos
        c = x.dot(x)-r**2
        if(c<0): return 0
        v = v0-target.vel
        a = v.dot(v)
        b = x.dot(v)
        if(b>0): return float('inf')
        discr = b*b -a*c
        if(discr<=0): return float('inf')
        tau = c/(-b +sqrt(discr))
        if(tau < 0): return float('inf')
        return tau

    def get_tc(self, cand_vnew, neighbor):
        temp = [0]*len(neighbor)
        for i in range(len(neighbor)):
            agent = neighbor[i]
            tau = self.get_tau(cand_vnew, agent)
            if(tau == 0): return 0
            else:
                temp[i] = tau
        return min(temp)

    def get_cost(self, cand_vnew, tc, alpha=1, beta=1, gama=2):
        return (alpha*(np.linalg.norm(cand_vnew-self.gvel)) + beta*(np.linalg.norm(cand_vnew-self.vel)) + (gama/tc) )

    def sample_vnew(self, n_sample):
        #temp_vnew = np.zeros((n_sample,2))
        temp_vnew = [[0,0]]*n_sample

        for i in range(0,n_sample):
            theta = random.random()*2*np.pi
            r = random.uniform(0,self.maxspeed)
            temp_vnew[i] = np.array([sin(theta)*(sqrt(r)), cos(theta)*(sqrt(r))])
        return temp_vnew

    def get_distance(self, target):
        distance = sqrt((self.pos[0]-target.pos[0])**2+(self.pos[1]-target.pos[1])**2) - (self.radius + target.radius)
        return distance
    
    def get_min_dist(self, neighbor):
        dist = [0]*len(neighbor)
        for i in range(len(neighbor)):
            dist[i] = self.get_distance(neighbor[i])
        return min(dist)

