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
from abc import ABC, abstractmethod 

class Planner(ABC):
    ''' Planner 
    '''

    @abstractmethod     
    # Used to specify an initialization policy
    def initialize_old_reference(self, p):
        pass
        #return r 
    
    @abstractmethod
    # Used to specify the actions to be performed when the reference has not yet been reached
    def compute_referecence_when_not_reached(self, p):
        pass
        #return [r, theta]

    @abstractmethod
    # Used to specify the actions to be performed to calculate a reference under normal use conditions
    def compute_standard_reference(self, p):
        pass
        #return [r, theta] 

    @abstractmethod
    # Method used to understand if a reference has been reached
    def reference_reached(self, p):
        pass
        #return res 

    def exit_from_recovery(self, p, r):
        res = abs(p[0] - r[0]) < self.tol or abs(p[1] - r[1]) < self.tol
        return res
       
    def need_recovery(self):
        res = self.is_recovery_enabled() and self.counter >= self.standstill
        return res
       
    def need_to_count_for_recovery(self, p):
        res = self.is_recovery_enabled() and norm(np.array(p) - np.array(self.p_old)) < self.rec_tolerance
        return res
       
    def is_recovery_enabled(self):
        # Check if recovery is active
        res = not(self.standstill == -1)
        return res
       

    def __init__(self, standstill = -1, tol = 0.1, counter = 0, rec_tolerance = 0.01, radius = 0.5):
        ''' 
            % Variable arguments
            % recovery  - if specified activate the recovery procedure
            %             if the vehicle does not move and requests 
            %             a new reference for standstill times, then the
            %             procedure is started.
            % rec_tolerance - tolerance used to understand if the vehicle
            %                 is stuck (meters)
            % rec_from_collision - used to choose between 2 different
            %                      recovery methods
            %                      (true recovery with anticollision
            %                      circular trajectory)
            %                      (false recovery with higher references)
            p_old                       % Old position of vehicle 
            r_old                       % Old reference
            radius                      % Radius for intermediate reference 
            %%% recovery properties
            is_in_recovery              % If the recovery reference has been reached or not
            counter                     % How much time the previous refence is still the same 
            rec_tolerance               % How many times you tolerate to stay still
            standstill                  % Dynamic value of old position freezing
            tol                         % Tolerance ([m]) used to understand if the vehicle has reached the reference
            recovery_from_collision     % To differentiate between Circular recovery or a custom one
        ''' 
        self.radius = radius
        self.p_old = None
        self.r_old = None
        self.is_in_recovery = False 
        self.standstill = standstill 
        self.tol = tol 
        self.counter = counter
        self.rec_tolerance = rec_tolerance


    def compute_reference(self, x, xa=[]):
        '''
            % Function used to compute a reference
            % x is the position -- Local Info
            % xa  - Augmented nieg states -- Neig Info 
        '''
        # Extract current state
        p = x
        if((self.p_old) is None): 
            self.p_old = p
        # Initialize the previous reference if necessary
        if(self.r_old is None):
            r_ = self.initialize_old_reference(p)
            self.r_old = r_ 




        # The reference is not updated if the current one has not been reached
        # If the vehicle does not reach the reference for too long a
        # recovery procedure is activated
        
        if(self.is_in_recovery): # planner in recovery mode (recovery reference not reached)
            #print("if") 
            r = self.r_old
            theta = 0 
            if(self.exit_from_recovery(p, r)):
                self.is_in_recovery = False
                self.counter = 0
        elif (not(self.reference_reached(p)) and not(self.need_recovery())): # standard reference not reached 
           # print("elif1") 
            # Reference does not change
            result = self.compute_referecence_when_not_reached(p)
            r = result[0]
            theta = result[1]
            if(self.need_to_count_for_recovery(p)): # Increase counter for recovery management
                self.counter = self.counter + 1
                
        elif (self.need_recovery()): # If the vehicle is stuck for whatever reason, start a recovery procedure
            #print("elif2") 
            result = self.compute_anticollision_recovery(p, xa)
            r = result[0]
            theta = result[1]
            r = p 
            theta = 0
            self.is_in_recovery = True 
            self.counter = 0 # reset the recovery counter

        else: # standart behavior, update the reference normally
        
            result = self.compute_standard_reference(p)        
            r = result[0]
            theta = result[1]
            # reset interanl state for recovery management
            self.counter = 0

        if(norm(np.array(p) - np.array(r[0:2])) > self.radius):
            r = self.inner_reference(p, r)
            
        self.r_old = list(r)
        self.p_old = list(p)
        r.append(theta) # [r,theta]
        return r


    def compute_anticollision_recovery(self, x, xa):
        #% Extract current state
        p = x
        # %%% Number of Neighboors
        return p

    def inner_reference(self, p, r): # % inherited abstract method
        ris = [0.0,0.0]

        if(r[0] - p[0] > 0):
            direction = 1
        else:
            direction = -1
        
        if((r[0] - p[0]) == 0):
            slope = 1000000
            intercept = p[0]
            if(r[1] - p[1] < 0):
                direction = 1
            else:
                direction = -1
        else:
            slope = (r[1] - p[1])/(r[0] - p[0])
            intercept = -slope*p[0] + p[1]

        [xout,yout] = self.linecirc(slope, intercept,p[0], p[1], self.radius)
        #% Choose the right poit to use as an intermediate reference
        if(slope > 100000 and xout != None and yout != None):
            if(direction < 0):
                if(yout[0] > yout[1]):
                    ris[0] = xout[0]
                    ris[1] = yout[0]
                else:
                    ris[0] = xout[1]
                    ris[1] = yout[1]
            else:
                if(yout[0] < yout[1]):
                    ris[0] = xout[0]
                    ris[1] = yout[0]
                else:
                    ris[0] = xout[1]
                    ris[1] = yout[1]
        else:
            if(direction < 0):
                if(xout[0] < xout[1]):
                    ris[0] = xout[0]
                    ris[1] = yout[0]
                else:
                    ris[0] = xout[1]
                    ris[1] = yout[1]
            else:
                if(xout[0] > xout[1]):
                    ris[0] = xout[0]
                    ris[1] = yout[0]
                else:
                    ris[0] = xout[1]
                    ris[1] = yout[1]
        return ris 

    def linecirc(self, slope,intercpt,centerx,centery,radius):
        #% find the cases of infinite slope and handle them separately

        if slope < 10000: 
        # % From the law of cosines

            a=1+slope**2
            b=2*(slope*(intercpt-centery)-centerx)
            c=centery*centery+centerx*centerx+intercpt**2-2*centery*intercpt-radius**2
            x = np.roots([a,b,c])

        # % Make NaN's if they don't intersect.

            if isinstance(x[0],complex) or isinstance(x[1], complex):
                x = None 
                y = None
            else:
                temp1 = np.array([intercpt, intercpt])
                temp2 = np.array([slope, slope])
                y = temp1 + temp2*x
        #% vertical slope case
        elif abs(centerx-intercpt)>radius:# % They don't intercept
            x = None 
            y = None
        else:
            x = np.array([intercept, intercept])
            step = np.sqrt(radius**2-(intercpt-centerx)**2)
            y = np.array([step+centery, -step+centery])


        return [x,y]
