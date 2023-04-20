#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 26 10:34:10 2022

@author: tanguy
"""

# External libs
import importlib
import pathlib 
import os
                
def simulate(config, design_choice = "baseline", id_config = None, solver_library_name = None, solver_name = None): 
    """
    Simulated a design and visualize it in SOFA GUI.
    Parameters
    ----------
    config: Config
        Config of the chosen problem
    design_choice: str among {baseline, from_optim, best}
        Select design to simulate
    id_config: str
        Id of the config
    solver_library_name: str
        Name of the solver library 
    solver_name: str
        Name of the solver from the solver library  
    """
    
    # Empty mesh directory
    for filename in os.listdir(config.base_meshes_path):
        if os.path.isfile(os.path.join(config.base_meshes_path, filename)):
            os.remove(os.path.join(config.base_meshes_path, filename))

    # Scene library
    scene_lib = importlib.import_module("Models." + config.model_name + "." + config.model_name)
    
    # Load baseline design
    if design_choice == "baseline":
        pass

    # Reload best design from simulation results
    elif design_choice == "best" or design_choice == "from_optim":

        # Init optimization problem
        if id_config != None:
            problem_name = config.model_name + "_" + id_config + "_" + solver_library_name + "_" + solver_name
        else:
            problem_name = config.model_name + "_" + solver_library_name + "_" + solver_name
        storage_name = "sqlite:///{}.db".format(str(pathlib.Path(__file__).parent.absolute())+"/OptimizationResults/" + config.model_name + "/" + problem_name)
        
        # Load solver library
        solver_lib = importlib.import_module("SolverLibraries."+ solver_library_name + ".SolverLibrary")
        solver = solver_lib.SolverLibrary(solver_library_name = solver_library_name, solver_name = solver_name)

        # Get parameters
        if design_choice == "best":
            parameters = solver.get_best_results(problem_name, storage_name, config)
        elif design_choice == "from_optim":
            parameters = solver.get_result_from_id(problem_name, storage_name, config)

        # Set parameters as design variables
        config.set_design_variables(list(map(list, parameters.items())))
        
    # Choose objectives to evaluate
    # In the future we may consider passing user-choosen objectives as arg parameters in main.py
    first_objective_name = list(config.get_objective_data().keys())[0]
    joint_objective_id = next((i for i,v in enumerate(config.get_assessed_together_objectives()) if first_objective_name in v), None)
    if joint_objective_id != None:
        config.set_currently_assessed_objectives(config.get_assessed_together_objectives()[joint_objective_id])
    else:
        config.set_currently_assessed_objectives([first_objective_name])
    
    # Launch design in SOFA
    import Sofa
    import Sofa.Gui    
    root = Sofa.Core.Node("root") # Generate the root node
    scene_lib.createScene(root, config) # Create the scene graph
    Sofa.Simulation.init(root) # Initialization of the scene graph
    
    # Find out the supported GUIs
    print ("Supported GUIs are: " + Sofa.Gui.GUIManager.ListSupportedGUI(","))
    Sofa.Gui.GUIManager.Init("myscene", "qglviewer")
    Sofa.Gui.GUIManager.createGUI(root, __file__)
    Sofa.Gui.GUIManager.SetDimension(1080, 1080)
    Sofa.Gui.GUIManager.MainLoop(root)
    
    
  
