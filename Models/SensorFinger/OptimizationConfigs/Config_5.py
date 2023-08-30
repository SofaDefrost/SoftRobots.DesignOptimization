# -*- coding: utf-8 -*-
"""Optimization config for the SensorFinger with less design variables and with adjustable cable position depending on cavity size.
Compared to optimization problem config 3, the used Poisson ratio and Finger mesh density are set respectively to 0.45 (instead of 0.3) and 3 (instead of 7).
The objective is to compare results obtained with this setting to see if mechanical parameters / mesh refinement impact greatly the results.
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

    def __init__(self):
        super().__init__()
        self.lc_finger = 3
        self.PoissonRation = 0.45


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
        return [["VolumeSensibility", "AbsoluteBendingAngle"]]
    
    def set_design_variables(self, new_values):
        super(OptimizationConfig,self).set_design_variables(new_values)
        self.CableHeight = self.OuterRadius + 0.8


    
    
