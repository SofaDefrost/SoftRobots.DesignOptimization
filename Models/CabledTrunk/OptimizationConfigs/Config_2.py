# -*- coding: utf-8 -*-
"""Reduced config for the Cabled Trunk
We optimize both for long and short cables which end tip is in the center of the Trunk.
"""

__authors__ = "tnavez"
__contact__ = "tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Jun 16 2023"


import sys
import pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute())+"/../")
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute()))

from Config import Config

import numpy as np 

class OptimizationConfig(Config):

    def __init__(self):
        super().__init__()
        self.var_cabled_modules =  3
        self.n_cables = 5
        self.n_short_cables = 2
        self.end_each_short_cable = [int(self.n_modules / 2), int(self.n_modules / 2)]

        self.init_cables()

    def get_objective_data(self):
        t = 40
        return {"ShapeMatchingBigS": ["minimize", t]
        }

    def get_assessed_together_objectives(self):
        return [["ShapeMatchingBigS"]]
    


    
    
