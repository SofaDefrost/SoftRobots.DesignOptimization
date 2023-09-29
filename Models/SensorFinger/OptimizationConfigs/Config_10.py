# -*- coding: utf-8 -*-
"""Optimization config for the SensorFinger with less design variables.
We optimise both for:
    - An absolute deflection angle.
    - A Volume Sensibility metric.
"""

__authors__ = "tnavez"
__contact__ = "tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Nov 25 2022"


import sys
import pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute())+"/../")
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute()))

from Config import Config

import numpy as np 

class OptimizationConfig(Config):

    def get_design_variables(self):            
        return {
        "JointSlopeAngle": [self.JointSlopeAngle, np.deg2rad(10), np.deg2rad(60)],   
        "OuterRadius": [self.OuterRadius, self.Thickness/2.0, self.Thickness/2.0 + 9.0],
        "JointHeight": [self.JointHeight, 5.0, 8.0],
        "CavityCorkThickness": [self.CavityCorkThickness, 2.0, 5.0],
        "BellowHeight": [self.BellowHeight, 6.0, 10.0],
        "WallThickness": [self.WallThickness, 2.0, 4.0],
        "PlateauHeight": [self.PlateauHeight, 2.0, 4.0]
        }

    def get_objective_data(self):
        return {"VolumeSensibility":["maximize", 80],
            "AbsoluteBendingAngle": ["maximize", 80],}

    def get_assessed_together_objectives(self):
        return [["VolumeSensibility"], ["AbsoluteBendingAngle"]]
    



    
    
