# -*- coding: utf-8 -*-
"""Reduced config for the Tripod Finger.
This is a scene with object for calibration purpose.
"""

"""
Same config as config 8, but the value of Poisson Coefficient in the main Config file should be 0.49 instead of 0.45
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

class OptimizationConfig(Config):

    def __init__(self):
        super().__init__()

    def get_design_variables(self):   
        return {"youngModulus": [self.youngModulus, 1.0e-3, 3.0e-2],
                "poissonRatio": [self.poissonRatio, 0.4, 0.499],
                "initTorque": [self.initTorque, 0.0, 0.2],
                "distanceObject": [self.distanceObject, 20.0e-3, 50.0e-3],
                }


    def get_objective_data(self):
        t = 80
        return {"TorqueCalibration": ["minimize", t]}

    def get_assessed_together_objectives(self):
        return [["TorqueCalibration"]]
    
    
