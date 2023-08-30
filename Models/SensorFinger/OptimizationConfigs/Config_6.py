# -*- coding: utf-8 -*-
"""Optimization config for simulating the prototyped Finger for RAL-2023."""

"""Optimization config for simulating the prototyped SensorFinger with less design variables and with adjustable cable position depending on cavity size.
We prototyped two designs from the Pareto Front of reduce config 3. 
Compared to Optimization config 3:
    - The used Poisson ratio is set to 0.495 (instead of 0.3) 
    - The mesh is thinner around cavities for ensuring having a better precision on the measure of the Volume Sensibility.
This Optimization config also enables sensitivity analysis around prototyped Sensor Finger for design parameters calibration purpose.
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

mode = "SENSOR"

class OptimizationConfig(Config):

    def __init__(self):
        super().__init__()
        if mode == "SENSOR":
            self.BellowHeight =  13
            self.OuterRadius = 17.522
            self.CavityCorkThickness = 2.048
            self.JointHeight = 6.450
            self.JointSlopeAngle = 0.592
            self.OuterRadius = 17.522
            self.PlateauHeight = 3.859
            self.WallThickness = 3
            self.CableHeight = self.OuterRadius + 1.0
        elif mode == "ANGLE":
            self.BellowHeight =  9.59572
            self.OuterRadius = 9.09886
            self.CavityCorkThickness = 2.142818
            self.JointHeight = 5.098034
            self.JointSlopeAngle = 0.87767209
            self.PlateauHeight = 3.838016
            self.WallThickness = 2.4453623
            self.CableHeight = self.OuterRadius + 1.0
        
        #self.WallThickness = 1.5
        self.lc_finger = 5
        self.RefineAroundCavities = True
        self.PoissonRation = 0.495

    def get_objective_data(self):
        return {"VolumeSensibility":["maximize", 80],
            "AbsoluteBendingAngle": ["maximize", 80],}

    def get_assessed_together_objectives(self):
        return [["VolumeSensibility", "AbsoluteBendingAngle"]]
        
    def get_design_variables(self):            
         return {
         "JointSlopeAngle": [self.JointSlopeAngle, 0.55, 0.65],   
         "OuterRadius": [self.OuterRadius, 17.0, 18.0],
         "JointHeight": [self.JointHeight, 6.0, 7.0],
         "CavityCorkThickness": [self.CavityCorkThickness, 1.5, 2.5],
         "BellowHeight": [self.BellowHeight, 10, 16.0],
         "WallThickness": [self.WallThickness, 2.0, 4.0],
         "PlateauHeight": [self.PlateauHeight, 3.6, 4.1]
         }

    def set_design_variables(self, new_values):
        super(OptimizationConfig,self).set_design_variables(new_values)
        self.CableHeight = self.OuterRadius + 1.0

    
    
