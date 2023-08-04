# -*- coding: utf-8 -*-
"""Main file to launch scripts for heuristic-based design optimization.
"""

__authors__ = "tnavez"
__contact__ = "tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Oct 28 2022"

# System libs
import argparse
import pathlib
import importlib


def main(args=None):
    """ Main entry point for the project
    Parameters
    ----------
    args : list
        A list of arguments as if they were input in the command line. Leave it to None to use sys.argv.
    """

    #######################
    ### Parse arguments ###
    #######################
    parser = argparse.ArgumentParser('Process args')

    ### Choose model
    parser.add_argument('--name', '-n', help='Load a model: -n model_name')
    parser.add_argument('--reduced_problem', '-rp', help='Identification number of the reduced problem for design optimization: -rp id.', default=None)

    ### Choose application and its parameters

    # Sensitivity Analysis
    parser.add_argument('--sensitivity_analysis', '-sa', help='Compute sensitivity analysis: -sa', action='store_true')
    parser.add_argument('--n_samples_per_param', '-nsa', help='Number of samples per optimization parameter for sensitivity analysis: -nsa n_samples_per_param', default= 2)
    parser.add_argument('--sa_method', '-sam', help='Method for sensitivity analysis: -sam sa_method', default= "OAaT") 
    implemented_sa_methods = ["OAaT", "Sobol", "Exhaustive"] # One-at-a-time (OAaT) strategy, variance-based (Sobol) or Exhaustive method

    # Design Optimization
    parser.add_argument('--optimization', '-o', help='Launch design optimization: -o', action='store_true')
    parser.add_argument('--n_iter', '-ni', help='Number of design optimization iterations: -ni n_iter', default= 10)
    parser.add_argument('--solver_library', '-sl', help='Name of the solver used for design optimization: -sl solver_name.', default="optuna")
    parser.add_argument('--solver_name', '-sn', help='Name of the type of solver used for design optimization: -sn solver_name.', default="evolutionary")
    implemented_solvers = {"optuna": ["evolutionary", "bayesian"]}
    
    # Design Simulation
    parser.add_argument('--simulate_design', '-sd', help='Simulate design: -sd. By default, simulate the baseline design.', action='store_true')
    parser.add_argument('--simulation_option', '-so', help='Simulation option: -so simulation_option. By default, baseline design option.', default='ba')
    simulation_options = ["ba", "fo", "be"] 

    # General application parameters
    parser.add_argument('--no_plot', '-np', help='Do not display produced plots: -np', action='store_true')
    
    # Parsing
    args = parser.parse_args(args)
    assert args.name != None, "Please enter a model name"
    config_link = pathlib.Path(str(pathlib.Path(__file__).parent.absolute())+"/Models/"+ args.name+"/Config.py")
    assert pathlib.Path.exists(config_link), "Please enter a valid model name"

    ####################
    ### Main operand ###
    ####################
    config_lib = importlib.import_module("Models."+ args.name+".Config")
    Config = config_lib.Config()
    id_config = None
    if args.reduced_problem:
        reduced_config_link = pathlib.Path(str(pathlib.Path(__file__).parent.absolute())+"/Models/"+ args.name+"/ReducedConfigs/Config_" + args.reduced_problem + ".py")
        if pathlib.Path.exists(reduced_config_link):
            print("Load reduced optimization config number " + args.reduced_problem)
            reduced_config_lib = importlib.import_module("Models."+ args.name+".ReducedConfigs.Config_" + args.reduced_problem)
            Config = reduced_config_lib.ReducedConfig()
            id_config = args.reduced_problem
        else:
            print("Please enter an existing reduced config number. Loading base config instead.")
    
    if args.solver_library not in implemented_solvers.keys():
            print("Please choose a valid solver library. Available solver library are " + str(list(implemented_solvers.keys())))
    elif args.solver_name not in implemented_solvers[args.solver_library]:
            print("Please choose a solver available in the solver library " + args.solver_library + ". Available solvers are " + str(implemented_solvers[args.solver_library]))

    if args.sensitivity_analysis: # Analyze the sensitivity of a set of design optimization objectives relative to each design variables.
        print("Starting sensitivity analysis.")
        sensitivity_analysis_lib = importlib.import_module("Applications.SensitivityAnalysis")
        if args.sa_method not in implemented_sa_methods:
            print("Please choose a valid sensitivity analysis method. Available sensitivity analysis methods are " + str(implemented_sa_methods))
        elif int(args.n_samples_per_param) < 0:
            print("Number of samples per parameter must be more than 0.")
        else:
            sensitivity_analysis_lib.analyse_sensitivity(Config, id_config=id_config, n_samples_per_param=int(args.n_samples_per_param), method=args.sa_method, plot_results=not args.no_plot)
    if args.optimization: # Optimize a design
        print("Starting design optimization.")
        optimization_lib = importlib.import_module("Applications.Optimize")
        if int(args.n_iter) < 0:
            print("Number of optimization iteration must be more than 0.")
        else:
            optimization_lib.optimize(Config, id_config=id_config, n_iter=args.n_iter, solver_library_name=args.solver_library, solver_name=args.solver_name, plot_results=not args.no_plot)
    if args.simulate_design: # Simulate design and visualize it in SOFA GUI
        print("Starting design simulation and visualization in SOFA GUI")
        simulate_lib = importlib.import_module("Applications.BasicSimulation")
        if args.simulation_option not in simulation_options:
            args.simulation_option = "ba"
        simulate_lib.simulate(Config, id_config=id_config, design_choice = args.simulation_option, solver_library_name=args.solver_library, solver_name=args.solver_name)       


if __name__ == "__main__":
    main()
