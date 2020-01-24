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
            #self.vnew[:] = self.gvel[:]   # here I just set the new velocity to be the goal velocity
            valid_neighbors = self.in_range(neighbors, 1.75)
            for i in valid_neighbors:
                if(self.id==i.id): valid_neighbors.remove(i)
            
            if(len(valid_neighbors)!=0):
                vnew_list = self.sample_vnew(100)
                self.vnew[:] = vnew_list[0]
                min_comp = pow((self.vnew[0]-self.gvel[0]),2) + pow((self.vnew[1]-self.gvel[1]),2)
                for cand_vnew in vnew_list:
                    for agent in valid_neighbors:
                        vel_comp = pow((cand_vnew[0]-self.gvel[0]),2) + pow((cand_vnew[1]-self.gvel[1]),2)
                        if((not self.test_vo(self.vnew, agent)) and vel_comp<min_comp):
                            min_comp = vel_comp
                            self.vnew[:] = cand_vnew
            else:
                self.vnew[:] = self.gvel[:]



      

    def update(self, dt):
        """ 
            Code to update the velocity and position of the agent  
            as well as determine the new goal velocity 
        """
        if not self.atGoal:
            self.vel[:] = self.vnew[:]
            self.pos += self.vel*dt   #update the position
        
            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                self.atGoal = True  # goal has been reached
            else: 
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed  

    def in_range(self, neighbors:list,threshold):
        valid_neighbors = neighbors.copy()
        for i in valid_neighbors:
            distance = sqrt(pow((self.pos[0]-i.pos[0]),2)+pow((self.pos[1]-i.pos[1]),2))
            if(distance>=threshold): valid_neighbors.remove(i)

        return valid_neighbors

    def get_vo(self, target):
        vo = [[0,0],[0,0]]

        temp_point = [(0+target.vel[0]),(0+target.vel[1])] #temp_point[0] = p, temp_point[1] = q
        circle_point = [target.pos[0]-self.pos[0], target.pos[1]-self.pos[1]] #circle_point[0] = a, circle_point[1] = b
        r = target.radius + self.radius

        p = temp_point[0]
        q = temp_point[1]
        a = circle_point[0]
        b = circle_point[1]

        #print(pow((p-a),2) + pow((q-b),2) - pow(r,2), 'id: ', self.id,' ', target.id)
        k1 = ((p-a)*(q-b) - r* sqrt(pow((p-a),2) + pow((q-b),2) - pow(r,2))) / (pow((p-a),2) - pow(r,2))
        vo[0][0] = k1
        vo[0][1] = q - p*k1

        k2 = ((p-a)*(q-b) + r* sqrt(pow((p-a),2) + pow((q-b),2) - pow(r,2))) / (pow((p-a),2) - pow(r,2))
        vo[1][0] = k2
        vo[1][1] = q - p*k2
        return vo

    def test_vo(self, test_v, target):
        vo = self.get_vo(target)
        y1 = test_v[0]*vo[0][0] + vo[0][1]
        y2 = test_v[0]*vo[1][0] + vo[1][1]
        return (test_v[1]>y1 and test_v[1]>y2)

    def sample_vnew(self, n_sample):
        temp_vnew = np.zeros((n_sample,2))

        for i in range(0,n_sample):
            theta = random.random()*2*np.pi
            r = random.uniform(0,self.maxspeed)
            temp_vnew[i][0] = sin(theta)*(sqrt(r))
            temp_vnew[i][1] = cos(theta)*(sqrt(r))
        return temp_vnew

    def get_distance(self, target):
        distance = sqrt((self.pos[0]-target.pos[0])**2+(self.pos[1]-target.pos[1])**2)
        return distance
        
  