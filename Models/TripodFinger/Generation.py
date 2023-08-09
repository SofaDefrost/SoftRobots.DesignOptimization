# -*- coding: utf-8 -*-
"""Shape generation for the SensorFinger"""

__authors__ = "bgouabau, tnavez, qpeyron"
__contact__ = "tanguy.navez@inria.fr, quentin.peyron@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Feb 09 2023"

import gmsh
import numpy as np
import locale
locale.setlocale(locale.LC_ALL, 'en_US.UTF-8')


##################
### Parameters ###
################## 
def define_parameters():
    mm = 1e-3
    ea1 = 8.0 * mm
    ea2 = 4.0 * mm
    La = 6.0 * mm
    la = 33.0 * mm
    inter = 15.2 * mm
    rHole = 1.7 * mm
    return ea1, ea2, La, la, inter, rHole


#######################
### Utility Volumes ###
#######################
# Generation of the volumes for clamping the flexible finger
def generate_volume_clamping(idx, n, lc):
    ea1, ea2, La, la, inter, rHole = define_parameters()

    # Definition of the points forming the contour of the clamped portion
    gmsh.model.occ.addPoint(0, -La, 0, lc, 1)
    gmsh.model.occ.addPoint(0, 0, 0, lc, 2)
    idxfin = 2 + 2 + 1 + n + 4 + n + 2
    gmsh.model.occ.addPoint(ea1, 0, 0, lc, idxfin + 1)
    gmsh.model.occ.addPoint(ea1, -La, 0, lc, idxfin + 2)

    # Definition of the points forming the contour of the fixed portion
    gmsh.model.occ.addPoint(la, 0, 0, lc, idx + 1)
    gmsh.model.occ.addPoint(la, -La, 0, lc, idx + 2)
    gmsh.model.occ.addPoint(la - ea2, -La, 0, lc, idx + 3)
    gmsh.model.occ.addPoint(la - ea2, 0, 0, lc, idx + 4)


def generate_holes_clamping(w, e1):
    ea1, ea2, La, la, inter, rHole = define_parameters()

    gmsh.model.occ.addCylinder(la, -La / 2, (w - inter) / 2, -ea2, 0, 0, rHole, 10)
    gmsh.model.occ.addCylinder(la, -La / 2, (w - inter) / 2 + inter, -ea2, 0, 0, rHole, 11)

    # Substract the holes for the fixation screws
    # Definition of the cylindrical holes for the fixation
    gmsh.model.occ.cut(e1, [(3, 10), (3, 11)])


#####################################
### Geometry Generation Functions ###
#####################################  
def TripodFinger(L, l, e1, e2, e3, n, d, w, lc):    
       
    # Generate the volume required to clamp the finger on the servo-motor
    idx = 1 + n + 4
    generate_volume_clamping(idx, n, lc)
    
    # Definition of the points forming the contour of the finger
    # Function addPOint(x,y,z,lc,tag)
    idx = 2
    gmsh.model.occ.addPoint(0, L, 0, lc, idx + 1)
    gmsh.model.occ.addPoint(l, L, 0, lc, idx + 2)
    gmsh.model.occ.addPoint(l, L-e2, 0, lc, idx + 3)
    for k in range(1, n + 1):
        gmsh.model.occ.addPoint(e1 + e3 + d[k - 1], L - e2 - k * (L - e2) / (n + 1), 0, lc, idx + 2 + 1 + k)
    
    idx = 2 + 2 + 1 + n + 4
    for k in range(1, n + 1):
        gmsh.model.occ.addPoint(e1 + d[n - k], k * (L - e2) / (n + 1), 0, lc, idx + k)
    idx = 2 + 2 + 1 + n + 4 + n
    gmsh.model.occ.addPoint(l - e3, L - e2, 0, lc, idx + 1)
    gmsh.model.occ.addPoint(e1, L - e2, 0, lc, idx + 2)
    
    gmsh.model.occ.synchronize()
    
    nbPoint = 2 + 2 + 1 + n + 4 + n + 2 + 2
    
    # Draw lines between the points
    for k in range(1, nbPoint):
        gmsh.model.occ.addLine(k, k + 1, k)
    
    gmsh.model.occ.addLine(nbPoint, 1, nbPoint)
    
    gmsh.model.occ.synchronize()
    
    # Define the contour
    gmsh.model.occ.addCurveLoop(range(1, nbPoint + 1), 1)
    
    # Create the surface delimited by the contour
    surf = gmsh.model.occ.addPlaneSurface([1])
    
    # Extrude the surface to obtain a volume
    # the extrude() function requires a vector pair (dimension of the object , tag of the object)
    ext1 = gmsh.model.occ.extrude([(2, surf)], 0, 0, w)
    
    # Generate the holes for the fixation of the finger with screws
    generate_holes_clamping(w, ext1)
    
    # Select all hexahedron elements for the discretization
    gmsh.option.setNumber("Mesh.SubdivisionAlgorithm", 0)
    
    gmsh.model.occ.synchronize()
    
    # Specify the mesh size
    gmsh.model.mesh.CharacteristicLengthMin = 0.001
    gmsh.model.mesh.CharacteristicLengthMax = 0.001

    # gmsh.model.mesh.generate(1)

    return surf


def ContactSurfaceIn1(L,e1, e2, w, lc):
    ea1, ea2, La, la, inter, rHole = define_parameters()
    
    gmsh.model.occ.addPoint(ea1, 0, 0, lc, 1)
    gmsh.model.occ.addPoint(e1, L - e2, 0, lc, 2)
    gmsh.model.occ.addPoint(e1, L - e2, w, lc, 3)
    gmsh.model.occ.addPoint(ea1, 0, w, lc, 4)

    # Draw liens between the points
    for k in range(1, 4):
        gmsh.model.occ.addLine(k, k + 1, k)
    gmsh.model.occ.addLine(4, 1, 4)

    # Define the contour
    gmsh.model.occ.addCurveLoop(range(1, 5), 1)

    # Create the surface delimited by the contour
    surf = gmsh.model.occ.addPlaneSurface([1])

    # # Mesh characteristics
    gmsh.model.mesh.CharacteristicLengthMin = 1
    gmsh.model.mesh.CharacteristicLengthMax = 1
    # gmsh.model.occ.synchronize()
    # gmsh.model.mesh.generate(1)

    return surf


def ContactSurfaceIn2(L, l, e1, e2, e3, n, d, w, lc):
    
    gmsh.model.occ.addPoint(l - e3, L - e2, 0, lc, 1)
    gmsh.model.occ.addPoint(l - e3, L - e2, w, lc, 2)

    for k in range(1, n + 1):
        gmsh.model.occ.addPoint(e1 + d[k - 1], L - e2 - k * (L - e2) / (n + 1), 0, lc, 2 * k + 1)
        gmsh.model.occ.addPoint(e1 + d[k - 1], L - e2 - k * (L - e2) / (n + 1), w, lc, 2 * k + 2)

    # Draw lines between the points
    for k in range(1, n + 1):
        gmsh.model.occ.addLine(2 * (k - 1) + 1, 2 * (k - 1) + 2, 4 * (k - 1) + 1)
        gmsh.model.occ.addLine(2 * (k - 1) + 2, 2 * (k - 1) + 4, 4 * (k - 1) + 2)
        gmsh.model.occ.addLine(2 * (k - 1) + 4, 2 * (k - 1) + 3, 4 * (k - 1) + 3)
        gmsh.model.occ.addLine(2 * (k - 1) + 3, 2 * (k - 1) + 1, 4 * (k - 1) + 4)

    surf_tags = []
    for k in range(1, n + 1):
        # Define the contour
        gmsh.model.occ.addCurveLoop(range(4 * (k - 1) + 1, 4 * (k - 1) + 5), k) #Define negative tags to ensure correct orientation

        # Create the surface delimited by the contour
        surf = gmsh.model.occ.addPlaneSurface([k])            
        surf_tags.append(surf)

    gmsh.model.occ.synchronize()

    # We finally generate and save the mesh
    #gmsh.model.mesh.generate(1)
    # Invert orientation 
    for surf_tag in surf_tags:
        gmsh.model.mesh.setReverse(2, surf_tag)
    
    return surf_tags


def ContactSurfaceOut(L, l, e1, e2, e3, n, d, w, lc):
    
    gmsh.model.occ.addPoint(l, L, 0, lc, 1)
    gmsh.model.occ.addPoint(l, L, w, lc, 2)
    gmsh.model.occ.addPoint(l, L-e2, 0, lc, 3)
    gmsh.model.occ.addPoint(l, L-e2, w, lc, 4)

    for k in range(1, n + 1):
        gmsh.model.occ.addPoint(e1 + e3 + d[k - 1], L - e2 - k * (L - e2) / (n + 1), 0, lc, 2 * (k+1) + 1)
        gmsh.model.occ.addPoint(e1 + e3 + d[k - 1], L - e2 - k * (L - e2) / (n + 1), w, lc, 2 * (k+1) + 2)

    # Draw lines between the points
    for k in range(1, n + 2):
        gmsh.model.occ.addLine(2 * (k - 1) + 1, 2 * (k - 1) + 2, 4 * (k - 1) + 1)
        gmsh.model.occ.addLine(2 * (k - 1) + 2, 2 * (k - 1) + 4, 4 * (k - 1) + 2)
        gmsh.model.occ.addLine(2 * (k - 1) + 4, 2 * (k - 1) + 3, 4 * (k - 1) + 3)
        gmsh.model.occ.addLine(2 * (k - 1) + 3, 2 * (k - 1) + 1, 4 * (k - 1) + 4)

    surf_tags = []
    for k in range(1, n + 2):
        # Define the contour
        gmsh.model.occ.addCurveLoop(range(4 * (k - 1) + 1, 4 * (k - 1) + 5), k);

        # Create the surface delimited by the contour
        surf_tags.append(gmsh.model.occ.addPlaneSurface([k]))

    gmsh.model.occ.synchronize()

    # Specify the mesh size, higher for contact surfaces to minimize the number of elements
    gmsh.model.mesh.CharacteristicLengthMin = 0.2
    gmsh.model.mesh.CharacteristicLengthMax = 0.2
    gmsh.model.mesh.generate(1)

    return surf_tags





