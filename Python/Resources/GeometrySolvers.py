# %% Kinematic Geometry Solvers
# This file has several functions that perform recurring geometry operations

import numpy as np

# %% Construct Plane from Three Points
def ThreePointPlane(p1, p2, p3, Form):
    # p1-3: Point Data
    # Form: Plane Return Form

    n = np.cross(p1-p2,p1-p3)
    n = n / np.linalg.norm(n)

    if Form == 'n':
        P = n
    elif Form == 'x':
        P = lambda y,z: -(n[1]*(y-p1[1]) + n[2]*(z-p1[2]))/n[0] + p1[0]
    elif Form == 'y':
        P = lambda x,z: -(n[0]*(x-p1[0]) + n[2]*(z-p1[2]))/n[1] + p1[1]
    elif Form == 'z':
        P = lambda x,y: -(n[0]*(x-p1[0]) + n[1]*(y-p1[1]))/n[2] + p1[2]

    return P

# %% Project Point onto Line or Plane
def PointProjection(Type, v0, p0, p):
    # Type : Projection Manifold Type (Line or Plane)
    # v0   : Line Tangent or Plane Normal
    # p0   : Point on Line or Plane
    # p    : Point to be Projected
    
    if Type == 'Line':
        proj = p0 + np.dot(v0,p-p0)/np.dot(v0,v0)*v0
    elif Type == 'Plane':
        dist = np.dot(v0,p) - np.dot(v0,p0)
        proj = p - dist*v0

    return proj

# %% Planar Intersection Line
def PlanarIntersection(p1, n1, p2, n2):
    d1 = np.dot(n1, p1)
    d2 = np.dot(n2, p2)

    n12 = np.dot(n1, n2)

    t = np.cross(n1, n2)
    t = t / t[0]

    p0 = (n1*(d1-d2*n12) + n2*(d2-d1*n12)) / (1-n12**2)
    p0 = p0 - p0[0]*t

