#  GNU License (C) 2025 LaSa, DIMES, Univerity of Calabria

# This program is developed for LaSa, DIMES, Univerity of Calabria. 
# Its copy, use, redistribution or modification is prohibited, or requires
# you to ask for permission. All authorized modifications made to 
# the software are subject to the same conditions as the original software.
# This program is provided as is: WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# For a copy of the complete license please contact elqemmah.ay@dimes.unical.it.

# Minimal notes added; program logic unchanged.

#! /usr/bin/env python3
import warnings
import numpy as np
from numpy.linalg import norm
import cvxpy as cp
import scipy.io
import math
import sys, os
from abc import ABC, abstractmethod 
sys.path.insert(0,os.path.expanduser('~')+'/ares_ros_ws/src/asv/lib/')
from planner.planner import Planner

class BorderPlanner(Planner):

    def __init__(self, limits, starting_slope,  standstill = -1, tol = 0.1, counter = 0, rec_tolerance = 0.01, radius = 0.5):
        super().__init__(standstill, tol, counter, rec_tolerance, radius)
        """
            properties
        limits          % A vector containing all points that have to be reached
        line                % Function of the strait line used for debug feature
        slope               % line slope
        intercept           % line y intercept (x intercept with inf slope)
        direction           % direction of movement (-1 from right to left, 1 from left to right)
        border_reference
        """ 
        self.point_iterator = 0
        self.direction = 1
        self.slope = starting_slope
        self.intercept = 0 
        self.border_reference = None # points
        self.limits = limits # [x_left, x_right, y_top, y_bottom]


    # Used to specify an initialization policy
    def initialize_old_reference(self, p):
        self.update_line(p, self.slope, self.direction)
        r = self.border_reference 
        theta = np.arctan2(r[1]-p[1], r[0]-p[0])

        return r 

    # Used to specify the actions to be performed when the reference has not yet been reached
    def compute_referecence_when_not_reached(self, p):
        r = self.r_old
        theta = np.arctan2(r[1] - p[1],r[0] - p[0])
        #print(self.r_old)
        #ris = np.array([r, theta])
        return (r, theta)

    # Used to specify the actions to be performed to calculate a reference under normal use conditions
    def compute_standard_reference(self, p):
        # compute m, q and check if it is necessary to change direction
        # if the vehicle has reached the left or right edge, the direction of movement must be changed
        
        if((abs(p[0] - self.limits[0])<self.tol or abs(p[0]-self.limits[1])<self.tol) and norm(np.array(p)-np.array(self.border_reference))<0.1):
            self.update_line(p, -self.slope, -self.direction)
            # if the vehicle has reached the upper or lower edge, it is not necessary to change the direction of movement
        elif((abs(p[1] - self.limits[2]) < self.tol or abs(p[1] - self.limits[3]) < self.tol) and norm(np.array(p) - np.array(self.border_reference)) < 0.1):
            self.update_line(p, -self.slope, self.direction)
        
        r = [self.border_reference[0], self.border_reference[1]]

        theta = np.arctan2(r[1]-p[1],r[0]-p[0])

        return (r, theta) 
        
    # Method used to understand if a reference has been reached
    def reference_reached(self, p):
        res = abs(p[0] - self.r_old[0]) < self.tol or abs(p[1] - self.r_old[1]) < self.tol
        return res 
    
    def update_line(self, p, slope, direction):
        self.intercept = p[1] - slope*p[0]
        self.slope = slope 
        if(slope < 100000):
            line = lambda x : slope*x + self.intercept
        else:
            line = lambda x : self.intercept
        self.direction = direction 
        inv_line = lambda y: ((y-self.intercept)/self.slope)
        # For the calculation of the reference we check which corner the vehicle is heading to
        if((self.direction > 0 and self.slope < 0) or ((self.direction < 0 and self.slope > 0))):
            #% lower right corner
            if((self.direction > 0 and self.slope < 0)):
                r = [self.limits[1], line(self.limits[1])]
                if(not(self.is_admissible(r))):
                    r = [inv_line(self.limits[2]), self.limits[2]]
            else: # % lower left corner
                r = [self.limits[0], line(self.limits[0])]
                if(not(self.is_admissible(r))):
                    r = [inv_line(self.limits[2]), self.limits[2]]
        else:
            #% upper left corner
            if((self.direction < 0 and self.slope < 0)):
                r = [self.limits[0], line(self.limits[0])]
                if(not(self.is_admissible(r))):
                    r = [inv_line(self.limits[3]), self.limits[3]]
            else: # % upper right corner
                r = [self.limits[1], line(self.limits[1])]
                if(not(self.is_admissible(r))):
                    r = [inv_line(self.limits[3]), self.limits[3]]
        print("UPDATE LINE")
        print(r)
        self.border_reference = r

    def is_admissible(self, r):
        # ris   - check result (true if r is inside the pool boundaries
        # false otherwise
        if((r[0] > self.limits[1]) or r[0] < self.limits[0]):
            resu = False 
        elif(r[1] > self.limits[3] or r[1] < self.limits[2]):
            resu = False
        else:
            resu = True 
        return resu 
        