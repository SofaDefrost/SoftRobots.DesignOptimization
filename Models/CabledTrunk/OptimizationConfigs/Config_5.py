# -*- coding: utf-8 -*-
"""Reduced config for the Cabled Trunk
Cable location optimization for following a target trajectory.
"""

__authors__ = "tnavez"
__contact__ = "tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Jun 16 2023"


import sys
import pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute())+"/../")
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute()))

from Config import Config

import numpy as np 

class OptimizationConfig(Config):

    def __init__(self):
        super().__init__()

        
        self.n_cables = 1 # 1 cable at the tip 

        self.var_cabled_modules =  3 # Number of modules where we can move the cable location
        k = 1 # Number of cable per considered intermediate module
        self.n_short_cables = 2 # 3 cables elsewhere
        self.end_each_short_cable += [8, 11] # Index of intermediate cable locations

        self.init_cables()

        self.use_contact = True


    def get_objective_data(self):
        t = 40
        return {"Trajectory": ["minimize", t]
        }
    

    def get_assessed_together_objectives(self):
        return [["Trajectory"]]

    
    
