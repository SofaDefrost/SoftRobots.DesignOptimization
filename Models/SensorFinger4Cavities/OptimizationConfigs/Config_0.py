# -*- coding: utf-8 -*-
"""Optimization config for a Sensorized Finger.
We optimise both for:
    - A pressure variation under forces applied on the Finger.
    - An absolute deflection angle i.e. the kinematics.
    - An altered Volume Sensibility metric for avoiding obtaining non feasible design with too small cavities
"""

__authors__ = "tnavez"
__contact__ = "tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Oct 28 2022"


import sys
import pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute())+"/../")
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute()))

from Config import Config

import numpy as np 

class OptimizationConfig(Config):

    def __init__(self):
        super().__init__()

        # Optimized parameters initialization
        self.JointSlopeAngle = np.deg2rad(30)
        self.OuterRadius = self.Thickness/2 + 6
        self.CableHeight = self.OuterRadius + 0.8
        self.JointHeight = 6
        self.BellowHeight = 5
        self.WallThickness = 3.5
        self.CenterThickness = 0.75

        # Modeling parameters
        self.lc_finger = 7
        self.RefineAroundCavities = True
        self.PoissonRation = 0.48

    def get_design_variables(self):            
        return {
        "JointSlopeAngle": [self.JointSlopeAngle, np.deg2rad(20), np.deg2rad(50)],   
        "OuterRadius": [self.OuterRadius, self.Thickness/2.0, self.Thickness/2.0 + 9.0],
        "JointHeight": [self.JointHeight, 5.0, 8.0],
        "BellowHeight": [self.BellowHeight, 4.0, 10.0],
        "CenterThickness": [self.CenterThickness, 0.75, 3.0],        
        }

    def get_objective_data(self):
        return {"PressureSensibility":["maximize", 160],
            "AbsoluteBendingAngle": ["maximize", 80],
            "InitialVolume": ["maximize", 1]}

    def get_assessed_together_objectives(self):
        return [["PressureSensibility", "AbsoluteBendingAngle", "InitialVolume"]]

    def set_design_variables(self, new_values):
        super(OptimizationConfig,self).set_design_variables(new_values)
        self.CableHeight = self.OuterRadius + 0.8

    


    
    
