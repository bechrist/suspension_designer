# %% BaseTesting
# This script tests a multibody suspension kinematic design suite by
# automatically designing and simulating a double wishbone suspension linkage 
# with a lower pushrod strut and droplink anti-roll bar (ARB).
#
# Blake Christierson (bechristierson@sbcglobal.net)
# 8 May 2022

import numpy as np

from KinematicSystem import *

# %% Targets
## Vehicle Targets (Static/Nominal)
Vehicle = dict()
Vehicle['CG'] = np.zeros(3)

Vehicle['Wheelbase']   = 1525                     # Wheelbase               [mm]
Vehicle['WeightDist']  = np.array([50.0,50.0])    # Weight Distribution      [%]
Vehicle['SprungMass']  = 225                      # Sprung Mass             [kg]
Vehicle['CG'][2]       = 8.50*(25.4)              # CG Height           [in->mm]
Vehicle['Ride']        = 2   *(25.4)              # Ride Height         [in->mm]
Vehicle['Rake']        = 0   *(np.pi/180)         # Rake Angle        [deg->rad]
Vehicle['LoadedRadius']= 7.85*(25.4)              # Loaded Radius       [in->mm]

## Suspension Type
Type = dict()

Type['Axle']           = "Front"                  # Axle
Type['Linkage']        = "Double Wishbone"        # Linkage Type
Type['Strut']          = "Lower Rod"              # Strut Type
Type['Spring']         = ["Ride","ARB"]           # Spring Type(s)

## Double Wishbone Targets
Linkage = dict()

Linkage['Track']       =  1220                    # Front Track Width       [mm]

Linkage['Toe']         =  0.50*(np.pi/180)        # Static Toe (+Out) [deg->rad]

Linkage['PitchCenter'] =  10                      # Normalized FBPC Height   [%]
Linkage['Caster']      =  3.00*(np.pi/180)        # Static Caster     [deg->rad]
Linkage['CasterGain']  =  0.25*(np.pi/(180*25.4)) # Caster Gain [deg/in->rad/mm]

Linkage['RollCenter']  =  15                      # Normalized FBRC Height   [%]
Linkage['Camber']      =- 1.60*(np.pi/180)        # Static Camber     [deg->rad]
Linkage['CamberGain']  =- 1.00*(np.pi/(180*25.4)) # Camber Gain [deg/in->rad/mm]

Linkage['Scrub']       =  0.50*(25.4)             # Maximum Scrub       [in->mm]
Linkage['KPI']         =  3.00*(np.pi/180)        # Target KPI        [deg->rad]

## Strut Targets
Strut = dict()

## Spring Targets
Spring = dict()

Spring['RideRatio']    =  0.80                    # Ride Ratio           [mm/mm]
Spring['ARBRatio']     =  0.80*(np.pi/(180*25.4)) # ARB Ratio   [deg/in->rad/mm]

## Compile Target Dictionary
Target = {'Vehicle': Vehicle, 
          'Type'   : Type   , 
          'Linkage': Linkage,
          'Strut'  : Strut  ,
          'Spring' : Spring }

del Vehicle, Type, Linkage, Strut, Spring 

# %% Bounds
# Notes:
# 1. Several bounds will be inherited if left to zero
#    - UAF inherits longitudinal bounds from LAF
#    - UAR inherits longitudinal bounds from LAR
#    - LAR inherits lateral bounds from LAF
#    - UAR inherits lateral bounds from UAF
# 2. Bounds set as NaN are fixed design parameters
#    - PA & SB should be within the y-z rocker plane
#    - PA sits on the y-axis of the rocker plane

Bound = dict()

## Double Wishbone Bounds
Linkage = dict()

# Inboard  Pickups:       | Longitudinal  |    Lateral    |    Vertical   |
Linkage['LAF'] = np.array([[  5.00,  5.00],[  8.00,  8.70],[  0.50,  1.50]]) # X
Linkage['LAR'] = np.array([[- 5.00,- 5.00],[  0   ,  0   ],[  0.50,  1.50]]) # X
Linkage['UAF'] = np.array([[  0   ,  0   ],[  8.70, 10.00],[  6.00,  8.00]]) # X
Linkage['UAR'] = np.array([[  0   ,  0   ],[  0   ,  0   ],[  6.00,  8.00]]) # X
Linkage['TA']  = np.array([[  2.00,  3.00],[  8.70,  8.70],[  2.50,  2.75]]) # X

# Outboard Pickups:       | Longitudinal  |    Lateral    |    Vertical   |
Linkage['LB']  = np.array([[  0.00,  0.00],[- 0.88,- 0.88],[- 3.25,- 2.70]]) # W
Linkage['UB']  = np.array([[  0.00,  0.00],[- 1.75,- 0.88],[  3.00,  3.50]]) # W
Linkage['TB']  = np.array([[  2.50,  2.85],[- 1.25,- 0.88],[- 1.50,  0.50]]) # W

## Strut Bounds
Strut = dict()

# Inboard  Pickups:    | Longitudinal  |    Lateral    |    Vertical   |
Strut['RA'] = np.array([[- 3.00,  2.00],[  6.00,  9.50],[  8.00, 10.00]]) # X
Strut['PA'] = np.array([[np.nan,np.nan],[  2.00,  4.00],[np.nan,np.nan]]) # R

# Outboard Pickups:    | Longitudinal  |    Lateral    |    Vertical   |
Strut['PB'] = np.array([[  2.25,  3.75],[- 3.00,- 2.00],[  1.50,  2.50]]) # A 

## Spring Bounds
Spring = dict()

# Inboard  Pickups:     | Longitudinal  |    Lateral    |    Vertical   |
Spring['SA'] = np.array([[- 3.00,  0.00],[  8.00, 12.00],[ 10.00, 18.00]]) # X

# Outboard Pickups:     | Longitudinal  |    Lateral    |    Vertical   |
Spring['SB'] = np.array([[np.nan,np.nan],[  2.25,  3.75],[  1.50,  2.50]]) # R

Bound = {'Linkage': Linkage, 
         'Strut'  : Strut  ,
         'Spring' : Spring }

# %% Linkage Design
## Initialize Linkage
Suspension = KinematicSystem('Test', Target, Bound)
## Sample Design Space

## Static Design Generation

# %% Linkage Simulation
## Jounce & Steer Sweep

a=1