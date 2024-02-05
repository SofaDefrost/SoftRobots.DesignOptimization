# -*- coding: utf-8 -*-
"""Reduced config for the Tripod Finger.
Test of an energy objective, with both transmission and cotnact force objectives.
Results are generated with calibrated data for FilaFlex Shore60.
"""

__authors__ = "tnavez"
__contact__ = "tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Feb 09 2023"


import sys
import pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute())+"/../")
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute()))

from Config import Config

import numpy as np 
import math

class OptimizationConfig(Config):

    def __init__(self):
        super().__init__()

        # More control points as the design is bigger
        self.n = 12
        for i in range(self.n):
            exec("self.d" + str(i) + "= 21.*self.mm")

        # More target angles for further control on the couple values
        self.n_target_angles = 40

        self.use_object = True    
        self.distanceObject = 33.1e-3

        # Calibrated mechanical parameters for FilaFlex Shore60
        self.youngModulus = 0.00362 # In GPa
        self.poissonRatio = 0.4518

        # Test calib
        # self.youngModulus = 0.00061365 # In GPa
        # self.poissonRatio = 0.456
        # self.max_angle = 4 * math.pi / 18 

        # Denser mesh, only for calibration purpose
        self.lc = 3 * self.mm


        
    def get_objective_data(self):
        t = 180
        return {"IncrementalForceTransmissionX": ["minimize", t],
                "ContactForceX": ["maximize", t],
                "GraspingEnergy": ["minimize", t]
                }

    def get_assessed_together_objectives(self):
        return [["IncrementalForceTransmissionX", "ContactForceX", "GraspingEnergy"]]
    
    def get_design_variables(self):   
        # Maximum bound variables
        self.e1max = 15 * self.mm
        self.e2max = 15 * self.mm
        self.e3max = 10 * self.mm
        self.delta= 20 * self.mm
        self.dmax = self.l + self.delta - self.e1max - self.e3max

        # Build design variables dictionnary
        design_variables = {
            "e1": [self.e1, 1*self.mm, self.e1max],
            "e2": [self.e2, 1*self.mm, self.e2max],
            "e3": [self.e3, 1*self.mm, self.e3max],
        }
        for i in range(self.n):
            design_variables["d" + str(i)] = [exec("self.d" + str(i)), 8*self.mm, self.dmax]
        return design_variables
