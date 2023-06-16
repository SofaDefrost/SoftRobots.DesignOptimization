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
        self.var_cabled_modules = 15 # Number of modules where we can move the cable location
                                    # This value should be self.n_modules for considering cable location variable everywhere
        
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

        ### Cables
        # High level params
        self.n_cables = 4
        self.dist_to_r_in = 1.0 * self.mm # Distance between cable attachment and internal radius

        # Attachment angles on each module for each cable
        angle = 2 * np.pi / self.n_cables # Angle between two points on the middle circle of a module
        for c in range(self.n_cables):
            for m in range(self.n_modules):
                exec("self.theta_" + str(c) + "_" + str(m) + " = " + str(c * angle))
            

    def get_design_variables(self):   
        # Build design variables dictionnary
        design_variables = {}

        ### Add angle variables
        for c in range(self.n_cables):
            for m in range(1, self.n_modules):
                design_variables["theta_" + str(c) + "_" + str(m)] = [exec("self.theta_" + str(c) + "_" + str(m)), 0, 2*np.pi]
        return design_variables

    def get_objective_data(self):
        t = 25 
        return {"ShapeMatching": ["minimize", t]
        }

    def get_assessed_together_objectives(self):
        return [["ShapeMatching"]]

    def set_design_variables(self, new_values):
        super(Config,self).set_design_variables(new_values) 

    
        