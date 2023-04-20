# -*- coding: utf-8 -*-
"""Base controller for evaluating the fitness of a design."""

__authors__ = "tnavez"
__contact__ = "tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Oct 28 2022"


import Sofa

class BaseFitnessEvaluationController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        """
        Classical initialization of a python class.

        Note:
        ----
            The kwargs argument must containe:
                name: str
                    Name of the controller
                config: Config 
                    Link to config of the model
                rootNode: Sofa.Node
                    Link to the root node of the Sofa scene
        """
        assert kwargs.get("name") == "FitnessEvaluationController"
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.rootNode = kwargs["rootNode"]
        self.config = kwargs["config"]
        self.eval_done = False
        self.objectives = []

    def get_computed_objectives(self):
        """
        Return computed objectives.
        Function to reimplement in the model scene controller.
        ----------
        Outputs
        ----------
        objective_name: str
            Name of the fitness function
        n_dt: int
            Number of dt steps for a the evaluation of the objective
        """
        if len(self.objectives) >= 1:
            return self.objectives
