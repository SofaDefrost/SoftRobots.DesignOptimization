# -*- coding: utf-8 -*-
"""Config for the TripodFinger"""

__authors__ = "tnavez"
__contact__ = "tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Feb 09 2023"


import sys
import pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute())+"/../")
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute()))

from BaseConfig import GmshDesignOptimization

import numpy as np 

class Config(GmshDesignOptimization):
    def __init__(self):
        super(GmshDesignOptimization,self).__init__("CabledTrunk")
        
    def init_model_parameters(self):
        ### Resolution mode
        self.inverse_mode = True

        ### Conversion to mm
        self.mm = 1e-3

        ### High level geometry parameters
        self.n_modules = 15 # Total number of modules
        self.var_cabled_modules = 14 # Number of modules where we can move the cable location
                                    # This value should be self.n_modules -1 for considering cable location variable 
                                    # everywhere but not on first module
        
        ### Modules geometry
        self.r_ext = 11 * self.mm # Radius of external circles describing a module
        self.d_ext = 4 * self.mm # Distance between internal and external circles of a module
        self.r_in = 15 * self.mm # Radius of internal circles describing a module
        self.d_in = 2 * self.mm # Distance between two internal circles of a module

        ### Linear size decreasing of modules
        self.min_radius_percent = 0.5
        self.min_module_size_percent = 0.85        

        ### Link between modules
        self.d_mspace = 3 * self.mm # Distance between two modules

        ### Cables ###
        # High level params        
        self.dist_to_r_in = 1.0 * self.mm # Distance between cable attachment and internal radius

        ### Cables going all the way to Trunk end tip
        self.n_cables = 4

        ### Shorter cables
        self.n_short_cables = 0 # Shorter cables which end point is not at the tip of the Trunk
        self.end_each_short_cable = [] # List of final modules for each short cables

        self.init_cables()        

    def init_cables(self):
        ### Cables going all the way to Trunk end tip
        # Attachment angles on each module for each cable
        if self.n_cables != 0:
            angle = 2 * np.pi / self.n_cables # Angle between two points on the middle circle of a module
            for c in range(self.n_cables):
                for m in range(self.n_modules):
                    exec("self.theta_" + str(c) + "_" + str(m) + " = " + str(c * angle))

        ### Shorter cables
        if self.n_short_cables != 0:
            angle_s = 2 * np.pi / self.n_short_cables # Angle between two points on the middle circle of a module
            for c in range(self.n_cables, self.n_cables + self.n_short_cables):
                for m in range(int(self.end_each_short_cable[c - self.n_cables])):
                    exec("self.theta_" + str(c) + "_" + str(m) + " = " + str((c - self.n_cables) * angle_s)) 

    def get_design_variables(self):   
        # Build design variables dictionnary
        design_variables = {}

        ### Add angle variables 
        # Usual length cables
        gap_var_cable_modules = int(self.n_modules / self.var_cabled_modules)
        ids_var_cable_modules = [self.n_modules - 1 - i*gap_var_cable_modules for i in range(self.var_cabled_modules)]
        for c in range(self.n_cables):
            for m in ids_var_cable_modules:
                design_variables["theta_" + str(c) + "_" + str(m)] = [exec("self.theta_" + str(c) + "_" + str(m)), 0, 2*np.pi]
        # Short cables
        for c in range(self.n_cables, self.n_cables + self.n_short_cables):
                gap_var_c_modules = int(self.end_each_short_cable[c - self.n_cables] / self.var_cabled_modules)
                ids_var_c_modules = [self.end_each_short_cable[c - self.n_cables] - 1 - i*gap_var_c_modules for i in range(self.var_cabled_modules)]
                for m in ids_var_c_modules:
                    design_variables["theta_" + str(c) + "_" + str(m)] = [exec("self.theta_" + str(c) + "_" + str(m)), 0, 2*np.pi]      
        return design_variables

    def get_objective_data(self):
        t = 25
        return {"ShapeMatchingBigS": ["minimize", t]
        }

    def get_assessed_together_objectives(self):
        return [["ShapeMatchingBigS"]]

    def set_design_variables(self, new_values):
        super(Config,self).set_design_variables(new_values) 

        ### We smooth the trajectory by using the shortest path to the closest imposed angle
        # Update non variable angles values to smooth usual length cables trajectories
        gap_var_cable_modules = int(self.n_modules / self.var_cabled_modules)
        ids_var_cable_modules = [self.n_modules - 1 - i*gap_var_cable_modules for i in range(self.var_cabled_modules)]
        ids_var_cable_modules.reverse()
        
        if ids_var_cable_modules[0] != 0:
            ids_var_cable_modules.insert(0, 0)
        pairs_var_cable_modules = [[ids_var_cable_modules[i-1], ids_var_cable_modules[i]]
                                   for i in range(1, len(ids_var_cable_modules))]

        for c in range(self.n_cables):
            for p in pairs_var_cable_modules:
                dist_angles = getattr(self, "theta_" + str(c) + "_" + str(p[1])) - getattr(self, "theta_" + str(c) + "_" + str(p[0]))
                signed_dist_angles = (dist_angles + np.pi) % (2 * np.pi) - np.pi
                angle_step = signed_dist_angles / (p[1] - p[0])

                for theta_id in range(p[0]+1, p[1]):
                    exec("self.theta_" + str(c) + "_" + str(theta_id) + " = " +
                         str(getattr(self, "theta_" + str(c) + "_" + str(p[0])) + angle_step * (theta_id - p[0])))

        # Update non variable angles values to smooth short length cables trajectories
        for c in range(self.n_cables, self.n_cables + self.n_short_cables):
            gap_var_c_modules = int(self.end_each_short_cable[c - self.n_cables] / self.var_cabled_modules)
            ids_var_c_modules = [self.end_each_short_cable[c - self.n_cables] - 1 - i*gap_var_c_modules for i in range(self.var_cabled_modules)]
            ids_var_c_modules.reverse()
            
            if ids_var_c_modules[0] != 0:
                ids_var_c_modules.insert(0, 0)
            pairs_var_c_modules = [[ids_var_c_modules[i-1], ids_var_c_modules[i]]
                                    for i in range(1, len(ids_var_c_modules))]
        
            for p in pairs_var_c_modules:
                dist_angles = getattr(self, "theta_" + str(c) + "_" + str(p[1])) - getattr(self, "theta_" + str(c) + "_" + str(p[0]))
                signed_dist_angles = (dist_angles + np.pi) % (2 * np.pi) - np.pi
                angle_step = signed_dist_angles / (p[1] - p[0])

                for theta_id in range(p[0]+1, p[1]):
                    exec("self.theta_" + str(c) + "_" + str(theta_id) + " = " +
                         str(getattr(self, "theta_" + str(c) + "_" + str(p[0])) + angle_step * (theta_id - p[0])))