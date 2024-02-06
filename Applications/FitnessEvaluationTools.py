# -*- coding: utf-8 -*-
"""Tools for fitness evaluation."""

__authors__ = "tnavez"
__contact__ = "tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Nov 07 2022"

# External libs
import importlib
import pathlib 
import sys
import Sofa

sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute())+"/../")  
  
# Simulation 
def simulation_loop(config, scene_lib):
    """
    Basic SOFA simulation loop for assessing a design on a particular task.
    ----------
    Inputs
    ----------
    config: Config
        Config class describing the optimization problem
    scene_lib: importlib link
        Link to the scene library.
    ----------
    Outputs
    ----------
    scores: list fo float
        Scores for the currently assessed objectives
    """

    # Find needed computation time
    current_objectives = config.get_currently_assessed_objectives()
    max_iter = max([config.get_objective_data()[current_objectives[i]][1] for i in range(len(current_objectives))])

    # Main SOFA loop
    root = Sofa.Core.Node("root") # Generate the root node     
    scene_lib.createScene(root, config) # Create the scene graph
    Sofa.Simulation.init(root) # Initialization of the scene graph
    for step in range(max_iter):
        Sofa.Simulation.animate(root, root.dt.value)
        Sofa.Simulation.updateVisual(root)
    scores = root.FitnessEvaluationController.objectives
    Sofa.Simulation.reset(root)
    return scores


def evaluate_fitness(config, scene_lib):
    """
    Manage simulation and objective computation for a set of sampled parameters
    ----------
    Inputs
    ----------
    config: Config
        Config class describing the optimization problem
    scene_lib: importlib link
        Link to the scene library.
    ----------
    Outputs
    ----------
    final_scores: list of float
        Score for each objective function
    """
    evaluated_objectives = []
    all_scores = {}
    for objective_name in config.get_objective_data():
        # Check objective has not already been evaluated
        if objective_name not in evaluated_objectives:
            # Check if objective can be assessed jointly with another objective and evaluate fitness functions accordingly
            joint_objective_id = next((i for i,v in enumerate(config.get_assessed_together_objectives()) if objective_name in v), None) # Find id of objective if in list
            if joint_objective_id != None:
                config.set_currently_assessed_objectives(config.get_assessed_together_objectives()[joint_objective_id])
                for joint_objective_name in config.get_assessed_together_objectives()[joint_objective_id]:
                    evaluated_objectives.append(joint_objective_name)
                    scores = simulation_loop(config, scene_lib)
                for i in range(len(config.get_assessed_together_objectives()[joint_objective_id])):
                    all_scores[config.get_assessed_together_objectives()[joint_objective_id][i]] = scores[i]
            else:
                config.set_currently_assessed_objectives(objective_name)
                all_scores[objective_name] = simulation_loop(config, scene_lib)[0]
    
    
    # Reorder results
    final_scores = []
    for objective_name in config.get_objective_data():
        final_scores.append(all_scores[objective_name])
    return final_scores
  
def wrap_evaluate_fitness(config):
    """
    Return a wrapped function that evaluate a geometry from an updated config.
    ----------
    Inputs
    ----------
    config: Config
        Config class describing the optimization problem
    scene_lib: importlib link
        Link to the scene library
    ----------
    Outputs
    ----------
    evaluate_fitness_wrapper: func
        Function that evaluate a geometry given a config.
    """
    scene_lib = importlib.import_module("Models." + config.model_name + "." + config.model_name)
    evaluate_fitness_wrapper = lambda config : evaluate_fitness(config, scene_lib) 
    return evaluate_fitness_wrapper