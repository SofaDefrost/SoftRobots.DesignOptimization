# -*- coding: utf-8 -*-
"""Shape generation for the Cabled Trunk"""

__authors__ = "tnavez"
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
    pass


#######################
### Utility Volumes ###
#######################
# Generate module parts of the continuous body 
def generate_module(r_ext, d_ext, r_in, d_in, translation):

    # Compute center of each circle 
    center_module = np.array([0,0,0])
    center_ext_1 = center_module + np.array([0.0, 0.0, -d_ext-(d_in/2)]) + translation
    center_in_1 = center_module + np.array([0.0, 0.0, -(d_in/2)]) + translation
    center_in_2 = center_module + np.array([0.0, 0.0, (d_in/2)]) + translation
    center_ext_2 = center_module + np.array([0.0, 0.0, +d_ext+(d_in/2)]) + translation
    
    # Create circles describing the volume
    circle_ext_1 = gmsh.model.occ.addCircle(*center_ext_1, r_ext)
    loop_ext_1 = gmsh.model.occ.addCurveLoop([circle_ext_1])
    circle_in_1 = gmsh.model.occ.addCircle(*center_in_1, r_in)
    loop_in_1 = gmsh.model.occ.addCurveLoop([circle_in_1])
    circle_in_2 = gmsh.model.occ.addCircle(*center_in_2, r_in)
    loop_in_2 = gmsh.model.occ.addCurveLoop([circle_in_2])
    circle_ext_2 = gmsh.model.occ.addCircle(*center_ext_2, r_ext)
    loop_ext_2 = gmsh.model.occ.addCurveLoop([circle_ext_2])

    # Create module volume
    volume = gmsh.model.occ.addThruSections([loop_ext_1, loop_in_1, loop_in_2, loop_ext_2])

    gmsh.model.occ.synchronize()

    return volume, loop_ext_1, loop_ext_2



#####################################
### Geometry Generation Functions ###
#####################################  
def Trunk(n_modules, r_ext, d_ext, r_in, d_in, min_radius_percent, min_module_size_percent,
          d_mspace, is_collision = False):    

    # Generate modules and links describing the Trunk
    module_volumes = []
    link_volumes = []
    last_loop_ext_2 = None
    dist_Z = 0
    for i in range(n_modules):
        # Generate module volume
        r_scaling_factors = 1.0 - i * (1.0 - min_radius_percent) / n_modules
        d_scaling_factor = 1.0 - i * (1.0 - min_module_size_percent) / n_modules
        module_length = d_scaling_factor * (d_ext + d_in + d_ext)
        translation = np.array([0, 0, dist_Z])
        module_volume, curr_loop_ext_1, curr_loop_ext_2  = generate_module(r_scaling_factors * r_ext, d_scaling_factor * d_ext, 
                                                                           r_scaling_factors * r_in, d_scaling_factor * d_in, 
                                                                           translation)
        module_volumes.append(module_volume)
        dist_Z += module_length + d_mspace

        # Generate link to previous module
        if not is_collision:
            if last_loop_ext_2 is not None:
                volume = gmsh.model.occ.addThruSections([last_loop_ext_2, curr_loop_ext_1])
                link_volumes.append(volume)
            
            last_loop_ext_2 = curr_loop_ext_2
    gmsh.model.occ.synchronize()

    # Merge volumes together
    trunk_volume = module_volumes[0]
    for i in range(1,n_modules):
        # Union with link
        if not is_collision:
            trunk_volume = gmsh.model.occ.fragment(trunk_volume, link_volumes[i-1])[0]
        # Union with next module
        trunk_volume = gmsh.model.occ.fragment(trunk_volume, module_volumes[i])[0]
    gmsh.model.occ.synchronize()

    # Homogeneize mesh
    gmsh.option.setNumber("Mesh.CharacteristicLengthMin", 0.001)  # Minimum mesh size
    gmsh.option.setNumber("Mesh.CharacteristicLengthMax", 0.01)  # Maximum mesh size
    gmsh.option.setNumber("Mesh.CharacteristicLengthFactor", 0.8) # Mesh size factor
    

    # gmsh.fltk.run()  
    return trunk_volume




######################
### Corridor plane ###
######################
def Corridor(x_scaling, y_scaling, z_scaling):    
    box = gmsh.model.occ.addBox(0, 0, 0, x_scaling, y_scaling, z_scaling)  
    return box