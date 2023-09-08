# -*- coding: utf-8 -*-
"""Script for managing sensitiity analysis of a design."""

__authors__ = "tnavez"
__contact__ = "tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Nov 07 2022"

# External libs
import importlib
import pathlib 
import os
import sys
import numpy as np
from .FitnessEvaluationTools import wrap_evaluate_fitness


def analyse_sensitivity(config, id_config, n_samples_per_param, method = "OAaT", plot_results = True): 
    """
    Local sensitivity analysis to find positive or negative effects of design parameters on objective.
    The local sensitivity is computed by 
            100 * (objective_max - objective_min)_local / (objective_max - objective_min)_global
            where: 
                (objective_max - objective_min)_local is computed when one output varies and others are assumed to be constant
                (objective_max - objective_min)_global is computed when all input varies 
    ----------
    Inputs
    ----------
    config: Config
        Config of the chosen problem
    id_config: str
        Id of the config
    n_samples_per_param: int
        Number of values to sample for each design variable  
    method: str in {OAaT, Exhaustive, Variance}
        Name of the method used for sensitivity analysis
            OAaT: one-at-a-time strategy with directive sensibilities giving a quick feeling of objective sensitivity to design variables but lacking to capture interactivity between variables.
                  It does not scale well when considering many design variables.
            Sobol: variance-based sensitivity analysis
            Exhaustive: grid sampling over the design space capturing interactivity between design variables but taking a long time to compute.
    plot_results: boolean
        Plot results
    """

    sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute())+"/../")  

    # Config
    if id_config == None:
        config_lib = importlib.import_module("Models."+ config.model_name +".Config")
    else:
        config_lib = importlib.import_module("Models."+ config.model_name + ".OptimizationConfigs.Config_" + str(id_config))   
    config.set_cache_mode(in_optimization_loop = True)

    # Simulation
    evaluate_fitness_wrapper = wrap_evaluate_fitness(config)
       
    ############################
    ### Sensitivity Analysis ###
    ############################

    # Common variables
    design_variables = config.get_design_variables()
    n_objectives = len(config.get_objective_data())
    name_objectives = list(config.get_objective_data().keys())
    objective_directions = [config.get_objective_data()[obj][0] for obj in config.get_objective_data()]
    name_variables = list(design_variables.keys())
    init_new_design_variables = [[name_var, design_variables[name_var][0]] for name_var in design_variables]   
              
    ### Compute local objectives when varying one parameter 
    if method == "OAaT":    
        objectives_vars = []
        id_var = 0
        rejected_var_pile = {} # Storing rejected variables corresponding to undefined geometries in shape {var_name : [value, replacement_value]}
        for name_var in design_variables: 
            # Reload config to reinit its design variables
            if id_config == None:
                config = config_lib.Config() 
            else:
                config = config_lib.OptimizationConfig()
            var_samples = list(np.linspace(design_variables[name_var][1], design_variables[name_var][2], n_samples_per_param))
            objectives_var = []
             # Compute objective for each sampled var
            for sample in var_samples:
                new_design_variables = init_new_design_variables
                new_design_variables[id_var][1] = sample
                config.set_design_variables(new_design_variables)

                # Create geometry and evaluate it
                is_geometry_valid = False
                k = 0.05
                while not is_geometry_valid:
                    try:
                        objectives = evaluate_fitness_wrapper(config)
                    except: #  Exception   
                        print("[ERROR] >> The geometry is not properly generated for " + name_var + " with value " + str(sample) + ".")
                        if sample < design_variables[name_var][1] + 0.5 * (design_variables[name_var][2] - design_variables[name_var][1]):
                            new_design_variables[id_var][1] = new_design_variables[id_var][1] + 0.1 * (design_variables[name_var][2] - design_variables[name_var][1])
                        else:
                            new_design_variables[id_var][1] = new_design_variables[id_var][1] - 0.1 * (design_variables[name_var][2] - design_variables[name_var][1])
                        config.set_design_variables(new_design_variables)
                        k += 0.05
                        print("\n Testing with value "  + str(new_design_variables[id_var][1]) + " for " + name_var + " instead.")
                    else:
                        is_geometry_valid = True
                        if k != 0.1:
                            if name_var in rejected_var_pile:
                                rejected_var_pile[name_var].append([sample, new_design_variables[id_var][1]])
                            else:
                                rejected_var_pile[name_var] = [[sample, new_design_variables[id_var][1]]]

                # Save fitness function(s)               
                objectives_var.append(objectives)
            objectives_vars.append(objectives_var)
            id_var += 1

        # Compute sensitivity metric for each design variable
        sensitivities = []
        global_sensitivities = []
        for i in range(n_objectives):
            objs_i_global = [[objectives_vars[j][k][i] for k in range(len(objectives_vars[j]))] for j in range(len(objectives_vars))]
            min_obj = min(map(min, objs_i_global))
            max_obj = max(map(max, objs_i_global)) 
            global_sensitivities.append(max_obj - min_obj)         
            
        for i in range(n_objectives):  
            sensitivities_i = []
            for j in range(len(objectives_vars)):
                objs_i_local = [objectives_vars[j][k][i] for k in range(len(objectives_vars[j]))] 
                sensitivities_i.append( 100 * (objs_i_local[-1] - objs_i_local[0]) / global_sensitivities[i])
            sensitivities.append(sensitivities_i) 

        # Display values
        print("\nSensitivity analysis done with method " + method + ":")
        for k in range(n_objectives):  
            print('\nObjective '+ name_objectives[k] + ':')
            print("\n".join('{}: {}'.format(i,j) for i, j in zip(name_variables, sensitivities[k])))

        # Display rejected variables
        print("\nUndoable geometries found for the following variables:")
        print("\n".join('{}: {} --> {}'.format(name_var,rejected_var_pile[name_var][i][0], rejected_var_pile[name_var][i][1]) for name_var in rejected_var_pile for i in range(len(rejected_var_pile[name_var]))))
        print("\nPlease consider changing the bounds of the above mentioned design variables.")
               
        # Plot results
        if plot_results:    

            def plot_sensitivities(sensitivities):
                import matplotlib.pyplot as plt     
                width = 0.2    
                fig, ax = plt.subplots() 
                for i in range(len(sensitivities)):
                    normalization = sum(map(abs, sensitivities[i]))
                    height = [sentivity/normalization for sentivity in sensitivities[i]]
                    #print("Normalization", normalization)
                    print("Height", height)
                    bars = name_variables
                    y_pos = np.arange(len(bars))
                    ax.bar(y_pos + i * width, height, width = width, label = name_objectives[i])
                    plt.xticks(y_pos, bars)
                plt.title("Local sensitivity (%)")
                plt.legend()
                fig.autofmt_xdate()
                if id_config != None:
                    name_file = config.model_name + "_" + str(id_config) + "_" + method + "_" + str(n_samples_per_param)
                else:
                    name_file = config.model_name + "_" + method + "_" + str(n_samples_per_param)

                plt.savefig(check_path(str(pathlib.Path(__file__).parent.absolute()) + "/SensitivityResults/" + config.model_name) + "/" + name_file)
                plt.show() 

            plot_sensitivities(sensitivities)

    ### Variance based sensitivity analysis
    elif method == "Sobol":
        
        # Implement first order indices method, measuring the effect of varying a parameter alone but averaged over variations in other input parameters
        pass


    ### Exhaustive sensitivity analysis
    # The method performs grid sampling over the design space for computing
    elif method == "Exhaustive":
        import pandas as pd
        from pandas.io.formats.style import Styler

        # Compute combinations of parameters
        vars_samples = []
        for name_var in design_variables:
            # Reload config to reinit its design variables
            if id_config == None:
                config = config_lib.Config() 
            else:
                config = config_lib.OptimizationConfig() 
            var_samples = list(np.linspace(design_variables[name_var][1], design_variables[name_var][2], n_samples_per_param))
            vars_samples.append(var_samples)
        combination_vars = [list(x) for x in np.array(np.meshgrid(*vars_samples)).T.reshape(-1,len(vars_samples))]
        
        # Compute objectives when varying parameters        
        objectives_vars = []
        for sample in combination_vars:
            new_design_variables = init_new_design_variables
            for id in range(len(sample)):
                new_design_variables[id][1] = sample[id]
            config.set_design_variables(new_design_variables)

            # Create geometry and evaluate it
            try:
                objectives = evaluate_fitness_wrapper(config)  
            except Exception:       
                print("[ERROR] >> The geometry is not properly generated for " + name_var + " with value " + str(sample) + ".")
                objectives_vars.append([])
            else:               
                objectives_vars.append(objectives)

        # Arrange results in a dataframe
        obj_params = [] # [parameter1, parameter2, ..., parameterN, objective1, ... objectiveM]
        for i in range(len(combination_vars)):
            if objectives_vars[i] != None:
                obj_params.append(combination_vars[i] + objectives_vars[i])
        df = pd.DataFrame(obj_params, columns= name_variables + name_objectives).astype('float')
              
        # Display and register best results
        styled_df = df.style
        for i in range(n_objectives): 
            if objective_directions[i] == "maximize":
                styled_df = styled_df.background_gradient(subset=name_objectives[i], cmap='RdYlGn')
            elif objective_directions[i] == "minimize":
                styled_df = styled_df.background_gradient(subset=name_objectives[i], cmap='GnYlRd')
        if id_config != None:
            name_file = config.model_name + "_" + str(id_config) + "_" + method + "_" + str(n_samples_per_param) + ".xlsx"
        else:
            name_file = config.model_name + "_" + method + "_" + str(n_samples_per_param) + ".xlsx"
        styled_df.highlight_null('lightblue').hide_index().to_excel(check_path(str(pathlib.Path(__file__).parent.absolute()) + "/SensitivityResults/" + config.model_name) + "/" + name_file, engine='openpyxl')


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
       
        
    
    
    
  
