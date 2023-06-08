# -*- coding: utf-8 -*-
"""Reduced config for the Tripod Finger.
We optimise both for:
    - A maximum contact Force along x-axis.
    - A minimum material mass needed to manufacture the finger.
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

class ReducedConfig(Config):

    def get_objective_data(self):
        return {"ContactForceX": ["maximize", 50],
        "Mass": ["minimize", 0]}

    def get_assessed_together_objectives(self):
        return [["ContactForceX", "Mass"]]
    


    
    
