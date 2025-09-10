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

class LinePlanner(Planner):

    def __init__(self, points,  standstill = -1, tol = 0.1, counter = 0, rec_tolerance = 0.01, radius = 0.5):
        super().__init__(standstill, tol, counter, rec_tolerance, radius)
        self.point_iterator = 0
        #points = np.array(points)
        #points = np.reshape(points, (points.shape[0], 1))
        self.points = points 
        self.direction = 1
        self.slope = 0
        self.intercept = 0 

    # Used to specify an initialization policy
    def initialize_old_reference(self, p):
        r = self.points[self.point_iterator:self.point_iterator+2]
        if(r[0] - p[0] > 0):
            self.direction = 1
        else:
            self.direction = -1
            
        if((r[0] - p[0]) == 0):
            self.slope = 1000000
            self.intercept = p[0]
            if(r[1] - p[1] < 0):
                self.direction = 1
            else:
                self.direction = -1
        else:
            self.slope = (r[1] - p[1])/(r[0] - p[0])
            self.intercept = -self.slope*p[0] + p[1]

        r = self.compute_next_reference(p)

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
        ref = self.points[self.point_iterator:self.point_iterator+2]
        
        if(norm(np.array(p) - np.array(ref)) > self.tol):  #% If the vehicle has reached an intermediate reference
            r = self.compute_next_reference(p)
            if(self.slope > 100000):
                if(self.direction > 0):
                    if(r[1] < ref[1]):
                        r = ref
                    
                else:
                    if(r[1] > ref[1]):
                        r = ref
            else:
                if(self.direction < 0):
                    if(r[0] < ref[0]):
                        r = ref
                else:
                    if(r[0] > ref[0]):
                        r = ref
            
        #% adjust the reference if it exceeds the final one   
        else: # % If the vehicle has reached the real reference
            self.point_iterator = self.point_iterator + 2; #% change reference
            if(self.point_iterator >= length(self.points)):
                self.point_iterator = 1
            r = self.initialize_old_reference(p) #% calculate another line
        theta = np.arctan2(r[1] - p[1],r[0] - p[0])
        return (r, theta) 

    # Method used to understand if a reference has been reached
    def reference_reached(self, p):
        res = abs(p[0] - self.r_old[0]) < self.tol or abs(p[1] - self.r_old[1]) < self.tol
        return res 
        
    def compute_next_reference(self, p):
        r = self.points[self.point_iterator:self.point_iterator+2]
        #% computentersections of circles and lines in Cartesian plane
        [xout,yout] = self.linecirc(self.slope, self.intercept, p[0], p[1], self.radius)
        if(xout is None or yout is None): #% if it can not be found compute a different line
            if(r[0] - p[0] > 0):
                self.direction = 1
            else:
                self.direction = -1
            if((r[0] - p[0]) == 0):
                self.slope = 1000000
                self.intercept = p[0]
                if(r[1] - p[1] < 0):
                    self.direction = 1
                else:
                    self.direction = -1
            else:
                self.slope = (r[1] - p[1])/(r[0] - p[0])
                self.intercept = -self.slope*p[0] + p[1]
            [xout,yout] = self.linecirc(self.slope, self.intercept,p[0], p[1], self.radius)
        
        #% Choose the right poit to use as an intermediate reference
        if(self.slope > 100000):
            print("IF")
            if(self.direction < 0):
                if(yout[0] > yout[1]):
                    r[0] = xout[0]
                    r[1] = yout[0]
                else:
                    r[0] = xout[1]
                    r[1] = yout[1]
            else:
                if(yout[0] < yout[1]):
                    r[0] = xout[0]
                    r[1] = yout[0]
                else:
                    r[0] = xout[1]
                    r[1] = yout[1]
        else:
            
            if(self.direction < 0):
                if(xout[0] < xout[1]):
                    r[0] = xout[0]
                    r[1] = yout[0]
                else:
                    r[0] = xout[1]
                    r[1] = yout[1]
            else:
                if(xout[0] > xout[1]):
                    r[0] = xout[0]
                    r[1] = yout[0]
                else:
                    r[0] = xout[1]
                    r[1] = yout[1]
        return r 