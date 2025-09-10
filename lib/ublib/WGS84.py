#  GNU License (C) 2025 LaSa, DIMES, Univerity of Calabria

# This program is developed for LaSa, DIMES, Univerity of Calabria. 
# Its copy, use, redistribution or modification is prohibited, or requires
# you to ask for permission. All authorized modifications made to 
# the software are subject to the same conditions as the original software.
# This program is provided as is: WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# For a copy of the complete license please contact elqemmah.ay@dimes.unical.it.

# Minimal notes added; program logic unchanged.

#! /usr/bin/env python
'''
WGS 84 - World Geodetic System standard to perform transformation between reference systems
Main reference systems:
    - LLA  :  latitude-longitude-altitude
    - ECEF :  earth centered earth fixed
    - ENU  :  east-north-up
'''

import numpy as np
import math

############# GEODETIC PARAMETERS #############

'''
WGS 84 PARAMETERS:
a                                     [m]    semimajor axis
b                                     [m]    semiminor axis
f      := (a-b)/a                     [-]    flattening factor
e      := sqrt(f*(2-f))               [-]    eccentricity factor
N(lat) := a/(sqrt(1-e^2*sin^2(lat)))  [m]    prime vertical radius
'''
a = 6378137
f = 1/298.257223563

e = np.sqrt(f*(2-f))
b = a*(1-f)
# Next lambda function have to receive LLA in [rad]
N = lambda lat: a/(np.sqrt(1-(e**2*np.sin(lat)**2)))
R = lambda lla: [[-np.sin(lla[1]),                 np.cos(lla[1]),                0],\
                 [-np.sin(lla[0])*np.cos(lla[1]), -np.sin(lla[0])*np.sin(lla[1]), np.cos(lla[0])],\
                 [ np.cos(lla[0])*np.cos(lla[1]),  np.cos(lla[0])*np.sin(lla[1]), np.sin(lla[0])] ]
#R = lambda lla: [[-np.sin(lla[0])*np.cos(lla[1]), -np.sin(lla[0])*np.sin(lla[1]), np.cos(lla[0])],\
#                 [ np.sin(lla[1]),                -np.cos(lla[1]),                0             ],\
#                 [ np.cos(lla[0])*np.cos(lla[1]),  np.cos(lla[0])*np.sin(lla[1]), np.sin(lla[0])] ]

############# REFERENCE SYSTEMS CONVERSION FUNCTIONS #############

def lla2ecef(lla):
    ''' Convert LLA[dd] to ECEF[m]
    ECEF_x := (N(lat)+alt)*cos(lat)*cos(lon)
    ECEF_y := (N(lat)+alt)*cos(lat)*sin(lon)
    ECEF_z := ((1-e^2)*N(lat)+alt)*sin(lat)
    '''
    ECEF_x = (N(np.radians(lla[0]))+lla[2])*np.cos(np.radians(lla[0]))*np.cos(np.radians(lla[1])) 
    ECEF_y = (N(np.radians(lla[0]))+lla[2])*np.cos(np.radians(lla[0]))*np.sin(np.radians(lla[1]))
    ECEF_z = ((1-e**2)*N(np.radians(lla[0]))+lla[2])*np.sin(np.radians(lla[0]))
    return (ECEF_x, ECEF_y, ECEF_z)


def ecef2lla(ecef):
    ''' Convert ECEF[m] to LLA[dd] '''
    #TODO any iterative methods
    return


def ecef2enu(ecef, lla_ref):
    ''' Convert ECEF[m] to ENU[m]
    Need a LLA reference frame respect to convert the point
    LLA_ref --> ECEF_ref --> [0,0,0]ned
    '''
    ecef_ref = lla2ecef(lla_ref)
    enu = np.dot(R(np.radians(lla_ref)),np.subtract(ecef,ecef_ref))
    return enu


def lla2enu(lla, lla_ref):
    ''' Convert LLA[dd] to ENU[m]
    Need a LLA reference frame respect to convert the point
    LLA_ref --> ECEF_ref --> [0,0,0]ned
    '''
    ecef_ref = lla2ecef(lla_ref)
    ecef = lla2ecef(lla)
    enu = np.dot(R(np.radians(lla_ref)),np.subtract(ecef,ecef_ref))
    return enu


def enu2lla(enu, lla_ref):
    ''' Convert ENU[m] to LLA[dd] '''
    #TODO
    return


def enu2ecef(enu, lla_ref):
    ''' Convert ENU[m] to ECEF[m] '''
    #TODO
    return


############# DEGREES CONVERSION FUNCTIONS #############

def dms2ddm(dms):
    ''' Convert Degrees-Minutes-Seconds to Degrees-Decimal Minutes
    DDD° MM' SS" --> DDD° MM.MMMMM'
    (d,m,s) --> (d,dm)
    '''
    if not len(dms) == 3: raise ValueError('Argument must be a list or tuple of 3 elements')
    d  = dms[0]
    dm = dms[1] + dms[2]/60.0
    return (int(d),round(dm,5))


def dms2dd(dms):
    ''' Convert Degrees-Minutes-Seconds to Decimal Degrees
    DDD° MM' SS" --> DDD.DDDDDDD°
    (d,m,s) --> (dd)
    '''
    if not len(dms) == 3: raise ValueError('Argument must be a list or tuple of 3 elements')
    dd = dms[0] + dms[1]/60.0 + dms[2]/3600.0
    return (round(dd,7))


def dd2dms(dd):
    ''' Convert Decimal Degrees to Degrees-Minutes-Seconds
    DDD.DDDDDDD° --> DDD° MM' SS"
    (dd) --> (d,m,s)
    '''
    if not len(dd) == 1: raise ValueError('Argument must be a list or tuple of 1 element')
    d = math.floor(dd[0])
    m = math.floor(60*(dd[0]-d))
    s = 3600*(dd[0]-d) - 60*m
    return (int(d),int(m),int(s))


def dd2ddm(dd):
    ''' Convert Decimal Degrees to Degrees-Decimal Minutes
    DDD.DDDDDDD° --> DDD° MM.MMMMM'
    (dd) --> (d,dm)
    '''
    if not len(dd) == 1: raise ValueError('Argument must be a list or tuple of 1 element')
    d  = math.floor(dd[0])
    dm = (dd[0]-d)*60
    return (int(d),round(dm,5))


def ddm2dd(ddm):
    ''' Convert Degrees-Decimal Minutes to Decimal Degrees
    DDD° MM.MMMMM' --> DDD.DDDDDDD°
    (d,dm) --> (dd)
    '''
    if not len(ddm) == 2: raise ValueError('Argument must be a list or tuple of 2 elements')
    dd = ddm[0] + ddm[1]/60.0
    return (round(dd,7))


def ddm2dms(ddm):
    ''' Convert Degrees-Decimal Minutes to Degrees-Minutes-Seconds
    DDD° MM.MMMMM' --> DDD° MM' SS"
    (d,dm) --> (d,m,s)
    '''
    if not len(ddm) == 2: raise ValueError('Argument must be a list or tuple of 2 elements')
    d = ddm[0]
    m = math.floor(ddm[1])
    s = (ddm[1]-m)*60
    return (int(d),int(m),int(s))
