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
        self.inverse_mode = False

        ### Conversion to mm
        self.mm = 1e-3

        ### High level geometry parameters
        self.Length = 250 * self.mm # Length of the Trunk 
        self.n_modules = 12 # Total number of modules
        self.cabled_modules = 5 # Number of modules where we can move the cable location


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
        self.n_cables = 4
        self.dist_to_r_in = 1.0 * self.mm # Distance between cable attachment and internal radius


    def get_design_variables(self):   
        return {"Length": [self.Length, 25.0, 35.0]
        }

    def get_objective_data(self):
        t = 50
        return {"ShapeMatching": ["minimize", t]
        }

    def get_assessed_together_objectives(self):
        return [["ShapeMatching"]]

    def set_design_variables(self, new_values):
        super(Config,self).set_design_variables(new_values) 

