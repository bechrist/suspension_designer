# %% Double Wishbone Test
# This script tests a multibody suspension kinematic design suite by
# automatically designing and simulating a double wishbone suspension.

import numpy as np

# %% Targets
Target         = dict()
Target['CG']   = np.zeros(3)
Target['Type'] = dict()

## Vehicle Targets
Target['Wheelbase']    = 1525             # Nominal Wheelbase               [mm]
Target['WeightDist']   = 0.5              # Front Weight Distribution         []
Target['SprungMass']   = 225              # Sprung Mass                     [kg]
Target['CG'][2]        = 8.50*(25.4)      # Nominal CG Height           [in->mm]
Target['Ride']         = 2   *(25.4)      # Nominal Ride Height         [in->mm]
Target['Rake']         = 0   *(np.pi/180) # Nominal Rake Angle        [deg->rad]
Target['LoadedRadius'] = 7.85*(25.4)      # Nominal Loaded Radius       [in->mm]

Target['CG'][0] = Target['Wheelbase']*(1-Target['WeightDist']) # CG to Axle [mm]

## Double Wishbone Targets

Target['Type']['Linkage'] = "Double Wishbone"    # Suspension Linkage Type

Target['Track']       =  1220                    # Front Track Width        [mm]

Target['Toe']         =  0.50*(np.pi/180)        # Static Toe (+Out)  [deg->rad]

Target['PitchCenter'] =  10                      # Normalized FBPC Height    [%]
Target['Caster']      =  3.00*(np.pi/180)        # Static Caster      [deg->rad]
Target['CasterGain']  =  0.25*(np.pi/(180*25.4)) # Caster Gain  [deg/in->rad/mm]

Target['RollCenter']  =  15                      # Normalized FBRC Height    [%]
Target['Camber']      = -1.60*(np.pi/180)        # Static Camber      [deg->rad]
Target['CamberGain']  = -1.00*(np.pi/(180*25.4)) # Camber Gain  [deg/in->rad/mm]

Target['Scrub']       =  0.50*(25.4)             # Maximum Scrub        [in->mm]
Target['KPI']         =  3.00*(np.pi/180)        # Target KPI         [deg->rad]

"""
Target['RideRatio']   =  0.80                    # Ride Motion Ratio Target   []
Target['ARBRatio']    =  0.80                    # ARB Motion Ratio Target    []
"""

# %% Double Wishbone Design Space Bounds
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

# Inboard  Pickups:     | Longitudinal  |    Lateral    |    Vertical   |
Bound['LAF'] = np.array([[  5.00,  5.00],[  8.00,  8.70],[  0.50,  1.50]]) # (X)
Bound['LAR'] = np.array([[- 5.00,- 5.00],[  0   ,  0   ],[  0.50,  1.50]]) # (X)
Bound['UAF'] = np.array([[  0   ,  0   ],[  8.70, 10.00],[  6.00,  8.00]]) # (X)
Bound['UAR'] = np.array([[  0   ,  0   ],[  0   ,  0   ],[  6.00,  8.00]]) # (X)
Bound['TA']  = np.array([[  2.00,  3.00],[  8.70,  8.70],[  2.50,  2.75]]) # (X)

"""
Bound['RA'] = np.array([[- 3.00,  2.00],[  6.00,  9.50],[  8.00, 10.00]]) # (X)
Bound['PA'] = [  NaN ,  NaN ;  2.00,  4.00;  NaN ,  NaN ]*(25.4) # (R)
Bound['SA'] = [- 3.00,  0.00;  8.00, 12.00; 10.00, 18.00]*(25.4) # (X)
"""

# Outboard Pickups:    | Longitudinal  |    Lateral    |    Vertical   |
Bound['LB'] = np.array([[  0.00,  0.00],[- 0.88,- 0.88],[- 3.25,- 2.70]]) # (W) 
Bound['UB'] = np.array([[  0.00,  0.00],[- 1.75,- 0.88],[  3.00,  3.50]]) # (W) 
Bound['TB'] = np.array([[  2.50,  2.85],[- 1.25,- 0.88],[- 1.50,  0.50]]) # (W)

"""
Bound['PB'] = [  2.25,  3.75;- 3.00,- 2.00;  1.50,  2.50]*(25.4) # (A) 
Bound['SB'] = [  NaN ,  NaN ;  2.25,  3.75;  1.50,  2.50]*(25.4) # (R)
"""