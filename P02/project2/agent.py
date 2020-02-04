# simulator.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the author.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#

import numpy as np
from math import sqrt

class Agent(object):

    def __init__(self, csvParameters, ksi=0.5, dhor = 10, timehor=5, goalRadiusSq=1, maxF = 10):
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
        self.goalRadiusSq = goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.ksi = ksi # the relaxation time used to compute the goal force
        self.dhor = dhor # the sensing radius
        self.timehor = timehor # the time horizon for computing avoidance forces
        self.F = np.zeros(2) # the total force acting on the agent
        self.maxF = maxF # the maximum force that can be applied to the agent

    def computeForces(self, neighbors=[], eps=0.3):
        """ 
            Your code to compute the forces acting on the agent. 
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors
        """       
        if not self.atGoal:
            self.F = (self.gvel-self.vel)/self.ksi #set F as F_goal
            valid_neighbors = self.in_range(neighbors, self.dhor) #get valid agents list within the dhor range
            '''
            k = 4
            n_nearest = min(k, len(valid_neighbors))
            dist = []
            for i in valid_neighbors:
                dist.append(self.get_distance(i))
            nearest_neighbors = ([x for _,x in sorted(zip(dist,valid_neighbors))])[:n_nearest] #get the nearest k agents list within the dhor range
            '''
            if(len(valid_neighbors)>0):
                for target in valid_neighbors:
                    self.F+=self.get_fa(target, eps)
                if (np.linalg.norm(self.F) > self.maxF):
                    self.F*=self.maxF/np.linalg.norm(self.F)
            else:
                self.F = (self.gvel-self.vel)/self.ksi
                

    def update(self, dt):
        """ 
            Code to update the velocity and position of the agents.  
            as well as determine the new goal velocity 
        """
        if not self.atGoal:
            self.vel += self.F*dt     # update the velocity
            self.pos += self.vel*dt   #update the position
        
            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            #distGoalSq = np.linalg.norm(self.pos-self.goal)
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
            
    def get_tau(self, v0, target, eps):
        r = self.radius+target.radius
        x = self.pos - target.pos
        c = x.dot(x)-r**2
        if(c<0): return 0
        v = v0-target.vel
        a = v.dot(v) - eps**2
        b = x.dot(v) - eps*r
        if(b>0): return float('inf')
        discr = b*b -a*c
        if(discr<=0): return float('inf')
        tau = c/(-b +sqrt(discr))
        if(tau < 0): return float('inf')
        return tau

    def get_fa(self, target, eps): #get avoid force of self and another one agent
        x = self.pos - target.pos
        v = self.vel - target.vel
        tau = self.get_tau(self.vel, target, eps)
        #n = (temp_avoid) / sqrt(temp_avoid[0]**2 + temp_avoid[1]**2)
        if(tau==0): return np.zeros(2)
        elif (tau==float('inf')): return np.zeros(2)
        else: 
            temp_avoid = x + v*tau
            n = (temp_avoid) / sqrt(temp_avoid[0]**2 + temp_avoid[1]**2)

        fa = (max(self.timehor-tau,0)/tau)*n
        return fa

    def get_distance(self, target):
        distance = sqrt((self.pos[0]-target.pos[0])**2+(self.pos[1]-target.pos[1])**2) - (self.radius + target.radius)
        return distance