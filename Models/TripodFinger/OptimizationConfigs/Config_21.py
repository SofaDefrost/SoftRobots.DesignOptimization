# -*- coding: utf-8 -*-
"""Reduced config for the Tripod Finger.
It is for calibration purpose, based on contact forces measured on the physical prototype.
We consider a specific design -> id = 203 from Reduced Config 19.
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
        self.n_target_angles = 10
        self.use_object = True    
        self.distanceObject = 33.1e-3
        self.max_angle = 4 * math.pi / 18 

        # Calibrated mechanical parameters for FilaFlex Shore60
        self.youngModulus = 0.00362 # In GPa
        self.poissonRatio = 0.4518

        # Design aprameters describing the considered design
        self.e1 = 14.7 * self.mm
        self.e2 = 3.2 * self.mm
        self.e3 = 9.7 * self.mm
        self.d0 = 12.2 * self.mm
        self.d1 = 17.3 * self.mm
        self.d2 = 13.1 * self.mm
        self.d3 = 15.1 * self.mm
        self.d4 = 18.1 * self.mm
        self.d5 = 28.0 * self.mm
        self.d6 = 20.9 * self.mm
        self.d7 = 9.0 * self.mm
        self.d8 = 18.3 * self.mm
        self.d9 = 19.3 * self.mm
        self.d10 = 10.6 * self.mm
        self.d11 = 18.1 * self.mm


        # Denser mesh, for calibration purpose
        self.lc = 2 * self.mm


    def get_design_variables(self):   
        return {"poissonRatio": [self.poissonRatio, 0.4, 0.499],
                "youngModulus": [self.youngModulus, 5.0e-4, 5.0e-2],
                }


    def get_objective_data(self):
        t = 60
        return {"ContactForceCalibration": ["minimize", t]}

    def get_assessed_together_objectives(self):
        return [["ContactForceCalibration"]]
      
  
