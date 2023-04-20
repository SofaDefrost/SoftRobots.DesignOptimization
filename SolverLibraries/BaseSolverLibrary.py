# -*- coding: utf-8 -*-
"""Base solver library class to reimplement for each solver library."""

__authors__ = "tnavez"
__contact__ = "tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Nov 02 2022"


class BaseSolverLibrary(object):

    def __init__(self, *args, **kwargs):
        """
        Classical initialization of a python class.

        Note:
        ----
            The kwargs argument must containe:
                solver_library_name: str
                    Name of the solver library
                solver_name: str
                    Name of the chosen solver from the solver_library
        """

        # Solver library name
        self.solver_library_name = kwargs["solver_library_name"]
        self.solver_name = kwargs["solver_name"]

    @staticmethod
    def get_all_solver_names(self):
        """
        Returns all implemented solver names.
        ----------
        Outputs
        ----------
        solver_names: list of str
            Name of solvers from the library
        """
        return []

    @staticmethod
    def optimize(self, problem_name, storage_name, config, n_iter, evaluate_fitness):
        """
        This function implements design optimization using provided solver and library.
        ----------
        Inputs
        ----------
        problem_name: str
            Name given to the optimization problem
        storage_name: str
            Path to the database where acquired data are stored
        config: Config
            Config class describing the optimization problem   
        n_iter: int
            Number of optimization iterations
        evaluate_fitness: func
            Function for evaluating the fitness function(s) for given design variables
        """
        return None
    
    @staticmethod
    def display_results(self, problem_name, storage_name, config):
        """
        This function display best results obtained from optimization.
        ----------
        Inputs
        ----------
        problem_name: str
            Name given to the optimization problem
        storage_name: str
            Path to the database where acquired data are stored
        config: Config
            Config class describing the optimization problem   
        """
        return None

    @staticmethod
    def plot_results(self, problem_name, storage_name, config):
        """
        This function plot optimization hsitory
        ----------
        Inputs
        ----------
        problem_name: str
            Name given to the optimization problem
        storage_name: str
            Path to the database where acquired data are stored
        config: Config
            Config class describing the optimization problem   
        """
        return None

    @staticmethod
    def get_result_from_id(self, problem_name, storage_name, config):
        """
        This function returns a chosen result obtained from an optimization problem.
        ----------
        Inputs
        ----------
        problem_name: str
            Name given to the optimization problem
        storage_name: str
            Path to the database where acquired data are stored
        config: Config
            Config class describing the optimization problem  
        ----------
        Outputs
        ----------
        parameters: list of duets [name, value] for each design variable
            Values for each design variables for the chosen design

        """
        return None

    @staticmethod
    def get_best_results(self, problem_name, storage_name, config):
        """
        This function returns best results obtained from an optimization problem.
        ----------
        Inputs
        ----------
        problem_name: str
            Name given to the optimization problem
        storage_name: str
            Path to the database where acquired data are stored
        config: Config
            Config class describing the optimization problem  
        ----------
        Outputs
        ----------
        best_parameters: list of duets [name, value] for each design variable
            Values for each design variables for the best design

        """
        return None