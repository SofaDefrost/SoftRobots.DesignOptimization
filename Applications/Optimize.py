# -*- coding: utf-8 -*-
"""Script for managing design optimization."""

__authors__ = "tnavez"
__contact__ = "tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Nov 02 2022"

# External libs
import os
import importlib
import pathlib 

from .FitnessEvaluationTools import wrap_evaluate_fitness

def optimize(config, id_config, n_iter, solver_library_name, solver_name, plot_results = True): 
    """
    Optimize design variables for a given problem
    ----------
    Inputs
    ----------
    config: Config
        Config of the chosen problem
    id_config: str
        Id of the config
    n_iter: int
        Number of design optimization iteration to perform
    solver_library_name: str
        Name of the solver library 
    solver_name: str
        Name of the solver from the solver library        
    plot: boolean
        Plot intermediate graph results
    """
    
    # Initialize multithreading feature
    config.set_cache_mode(in_optimization_loop = True)
    
    # Simulation
    evaluate_fitness_wrapper = wrap_evaluate_fitness(config)
       
    # Init optimization problem
    if id_config != None:
        problem_name = config.model_name + "_" + id_config + "_" + solver_library_name + "_" + solver_name
    else:
        problem_name = config.model_name + "_" + solver_library_name + "_" + solver_name
    storage_name = "sqlite:///{}.db".format(check_path(str(pathlib.Path(__file__).parent.absolute())+"/OptimizationResults/" + config.model_name) + "/" + problem_name)
       
    # Optimization using chosen solver library
    solver_lib = importlib.import_module("SolverLibraries."+ solver_library_name + ".SolverLibrary")
    solver = solver_lib.SolverLibrary(solver_library_name = solver_library_name, solver_name = solver_name)
    solver.optimize(problem_name, storage_name, config, n_iter, evaluate_fitness_wrapper)
    
    # Display best results
    solver.display_results(problem_name, storage_name, config)

    # Plot results
    if plot_results:      
        solver.plot_results(problem_name, storage_name, config)


def check_path(path):
    """
    Check if a path exists.
    Create necessary folders if not.
    ----------
    Input
    ----------
    path: str
        The adress to check.
    ----------
    Output
    ----------
    path: str
        The adress to check.
    """
    if not os.path.exists(path):
        os.makedirs(path)
    return path

       
        
    
    
    
  
