# -*- coding: utf-8 -*-
"""Config for the SensorFinger"""

__authors__ = "tnavez"
__contact__ = "tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Oct 28 2022"


import sys
import pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute())+"/../")
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute()))

from BaseConfig import GmshDesignOptimization

import numpy as np 

class Config(GmshDesignOptimization):
    def __init__(self):
        super(GmshDesignOptimization,self).__init__("SensorFinger")
        
    def init_model_parameters(self):

        ########################
        ### Parametric Model ###
        ########################

        # Geometric parameters
        self.Length = 40
        self.Height = 20
        self.JointHeight = 6
        self.Thickness = 17.5
        self.JointSlopeAngle = np.deg2rad(30)
        self.FixationWidth = 3
        
        self.OuterRadius = self.Thickness/2 + 6
        self.NBellows = 1
        self.BellowHeight = 8
        self.TeethRadius = self.Thickness/2   
        self.WallThickness = 3
        self.CenterThickness = 1.5
        self.CavityCorkThickness = 3
        self.PlateauHeight = 3
        
        # Elasticity parameters
        self.PoissonRation = 0.3 #0.47
        self.YoungsModulus = 3000

        # Meshing parameters
        self.lc_finger = 7
        self.RefineAroundCavities = False
        
        # Mold parameters
        self.MoldWallThickness = 3
        self.MoldCoverTolerance = 0.1
        self.LengthMold = 3*self.Length + 2*self.MoldWallThickness
        self.LidHoleBorderThickness = 1
        self.LidHoleThickness = self.Thickness - 2*self.LidHoleBorderThickness
        self.LidHoleLength = 3*self.Length/5
        
        self.MoldHoleThickness = self.Thickness - 2*self.LidHoleBorderThickness
        self.MoldHoleLength = self.Length/2
        
        self.ThicknessMold = 2*self.OuterRadius + 2*self.MoldWallThickness
        self.LengthMold = 3*self.Length + 2*self.MoldWallThickness
        self.HeightMold = self.Height + self.FixationWidth + self.MoldWallThickness    
        self.MoldHoleLidBorderThickness = 2
        
        # Cable
        self.CableRadius = 0.8
        self.CableDistance = 10
        self.CableHeight = 17.75
        
    def get_design_variables(self):            
        return {"Length": [self.Length, 20.0, 60.0], 
        "Height": [self.Height, 15.0, 25.0],
        "JointHeight": [self.JointHeight, 5.0, 8.0],
        "JointSlopeAngle": [self.JointSlopeAngle, np.deg2rad(10), np.deg2rad(60)],
        "FixationWidth": [self.FixationWidth, 2.5, 3.5],      
        "OuterRadius": [self.OuterRadius, self.Thickness/2.0, self.Thickness/2.0 + 9.0], 
        "BellowHeight": [self.BellowHeight, 6.0, 10.0],
        "TeethRadius": [self.TeethRadius, self.Thickness/2 - 1.0, self.Thickness/2 + 1.0],
        "WallThickness": [self.WallThickness, 2.0, 4.0],
        "CenterThickness": [self.CenterThickness, 1.0, 4.0],
        "CavityCorkThickness": [self.CavityCorkThickness, 2.0, 5.0],
        "PlateauHeight": [self.PlateauHeight, 2.0, 4.0]
        }
               
    def get_objective_data(self):
        return {"PressureSensibility": ["maximize", 80],
        "AbsoluteBendingAngle": ["maximize", 80],}

    def get_assessed_together_objectives(self):
        return [["PressureSensibility", "AbsoluteBendingAngle"]]

    def set_design_variables(self, new_values):
        super(Config,self).set_design_variables(new_values)

        # Update mold parameters
        self.MoldWallThickness = 3
        self.MoldCoverTolerance = 0.1
        self.LengthMold = 3*self.Length + 2*self.MoldWallThickness
        self.LidHoleBorderThickness = 1
        self.LidHoleThickness = self.Thickness - 2*self.LidHoleBorderThickness
        self.LidHoleLength = 3*self.Length/5
        
        self.MoldHoleThickness = self.Thickness - 2*self.LidHoleBorderThickness
        self.MoldHoleLength = self.Length/2
        
        self.ThicknessMold = 2*self.OuterRadius + 2*self.MoldWallThickness
        self.LengthMold = 3*self.Length + 2*self.MoldWallThickness
        self.HeightMold = self.Height + self.FixationWidth + self.MoldWallThickness    
        self.MoldHoleLidBorderThickness = 2
    
    
