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
import math

class Config(GmshDesignOptimization):
    def __init__(self):
        super(GmshDesignOptimization,self).__init__("TripodFinger")
        
    def init_model_parameters(self):
        # Specific scene parameters
        self.use_object = True # Specify if we have an object in the scene
        self.object_shape = "cylinder" # Object shape. Should be cylinder or sphere.
        self.n_target_angles = 8 # Number of intermediate angle objectives - Increasing this number is usefull for couple servoing
        self.max_angle = math.pi / 6 # Maximum angle impsoed on servomotor


        # Conversion to mm
        self.mm = 1e-3

        # Geometry parameters
        self.L = 100.* self.mm
        self.l = 33. * self.mm
        self.e1 = 8.* self.mm
        self.e2 = 5.* self.mm
        self.e3 = 4.* self.mm
        self.n = 7
        for i in range(self.n):
            exec("self.d" + str(i) + "= 21.*self.mm")
        
        # Thickness of the finger / length of extrusion
        self.w = 20.0 * self.mm

        # Mesh refinement parameter
        self.lc = 12 * self.mm
        self.lc = 5 * self.mm

        # Printing parameters
        self.rho = 500 # kg/m3

        # Simulation parameters
        self.youngModulus = 0.02997 # In GPa, optimized
        self.poissonRatio = 0.4713
        self.distanceObject = 47.8e-3

        # Sensing parameters
        self.initTorque = 0.0294 # in N.m, due to the pre-stress of the FSR sensor

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

    def get_objective_data(self):
        t = 50
        return {"ContactForceX": ["maximize", t], 
        "ContactForceNorm": ["maximize", t],
        "Mass": ["minimize", 0],
        "ForceTransmissionX": ["minimize", t],
        "InterpenetrationPenaltyInitX": ["minimize", 1],
        "InterpenetrationPenaltyEndX": ["minimize", t],
        "LocateContactPrecision": ["minimize", t],
        "TorqueCalibration": ["minimize", t]
        }

    def get_assessed_together_objectives(self):
        return [["ContactForceX", "ContactForceNorm", "Mass", "ForceTransmissionX", 
                 "InterpenetrationPenaltyInitX", "InterpenetrationPenaltyEndX", "LocateContactPrecision","TorqueCalibration"]]

    def set_design_variables(self, new_values):
        super(Config,self).set_design_variables(new_values) 

