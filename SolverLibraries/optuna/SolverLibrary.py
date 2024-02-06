# -*- coding: utf-8 -*-
"""Optuna solver library. Available solvers are:
        - bayesian: TPE (single objective) and MOTPE (multi objectives)
        - evolutionary: CmaEs (single objective) and NSGAII (multi objectives)
    The library manages optimization problems with continuous design variables bounded by box constraints."""

__authors__ = "tnavez"
__contact__ = "tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Nov 02 2022"


import sys
import pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute())+"/../")
sys.path.insert(0, str(pathlib.Path(__file__).parent.absolute()))

from BaseSolverLibrary import BaseSolverLibrary

import numpy as np 
import math
import optuna
import logging

class SolverLibrary(BaseSolverLibrary):
    def __init__(self, *args, **kwargs):
        super(SolverLibrary,self).__init__(*args, **kwargs)
        
    def get_all_solver_names(self):
        return ["evolutionary", "bayesian"]

    def optimize(self, problem_name, storage_name, config, n_iter, evaluate_fitness):   
        # Init optimization problem
        problem = self.init_problem(problem_name, storage_name, config)

        # Optimize
        self.evaluate_fitness = evaluate_fitness
        compute_objectives_wrapper = lambda trial: self.compute_objectives(trial, config)
        problem.optimize(compute_objectives_wrapper, n_trials = int(n_iter)) 

    def display_results(self, problem_name, storage_name, config):
        # Reload optimization problem
        problem = self.init_problem(problem_name, storage_name, config)

        # Display best results
        objectives_data = config.get_objective_data()
        n_objectives = len(objectives_data)
        if n_objectives == 1:
            trial = problem.best_trial
            print('Objective: {}'.format(trial.value))
            print('Best hyperparameters: {}'.format(trial.params)) 
        elif n_objectives > 1:
            import random
            k = max(5, len(problem.best_trials))
            print("For multi-objective optimization, a set of maximum " + str(k) + " of the best results are displayed at random.")
            best_trials = random.choices(problem.best_trials, k=k)
            print('Best Objectives: ' + ' '.join(map(str, [best_trials[i].values for i in range(len(best_trials))])))
            # print('Best hyperparameters: ' + ' '.join(map(str, [best_trials[i].params for i in range(len(best_trials))])))

    def plot_results(self, problem_name, storage_name, config):
        import matplotlib.pyplot as plt
        from matplotlib.pyplot import cm

        # Reload optimization problem
        problem = self.init_problem(problem_name, storage_name, config)
        
        # Retrieve trials history
        trials = problem.trials        
        n_complete = len([t for t in trials if t.state == optuna.structs.TrialState.COMPLETE])
        print('Num. of complete trials:', n_complete)        
        t = [i for i in range(n_complete)]
        objectives_data = config.get_objective_data()
        n_objectives = len(objectives_data)
        
        # Plot objectives convergence history
        # if n_objectives == 1:
        #     objectives = []
        #     for trial in trials:
        #         if trial.state == optuna.structs.TrialState.COMPLETE:
        #             objectives.append(trial.value)
        #     print("objectives:", objectives)
        #     plt.scatter(t, objectives, alpha=0.5)
        # elif n_objectives > 1:
        #     objectives_history = []
        #     colors = cm.rainbow(np.linspace(0, 1, n_objectives))
        #     for i in range(n_objectives):
        #         objectives = []
        #         for trial in trials:
        #             if trial.state == optuna.structs.TrialState.COMPLETE:
        #                 objectives.append(trial.values[i])
        #         objectives_history.append(objectives)
        #         if i != 0:
        #             plt.twinx()
        #         plt.scatter(t, objectives, alpha=0.5, color = colors[i])
        #         plt.tick_params(axis='y', labelcolor=colors[i])
        #         plt.ylabel(list(objectives_data.keys())[i])             
        # plt.suptitle("Objective per trial")
        # plt.show()

        # Objective history for single-objective optimization
        if n_objectives == 1:
            fig = optuna.visualization.plot_optimization_history(problem, target_name=list(objectives_data.keys())[0])
            fig.show()
        
        # Plot Paretto curve for multi-objective optimization
        if n_objectives == 2:     
            fig = optuna.visualization.plot_pareto_front(problem, target_names=[list(objectives_data.keys())[0], list(objectives_data.keys())[1]])
            fig.show()  

        elif n_objectives == 3:   
            target_names=[list(objectives_data.keys())[0], list(objectives_data.keys())[1], list(objectives_data.keys())[2]]
            fig = optuna.visualization.plot_pareto_front(problem, target_names=target_names)
            fig.show()
            
        if n_objectives >= 3:
            pass
            """
            # Optuna implementation
            import itertools
            objectives_combinations = list(itertools.combinations(list(range(len(objectives_data))), 2))
            for combination in objectives_combinations:
                target_names = [list(objectives_data.keys())[combination[0]], list(objectives_data.keys())[combination[1]]]
                targets = lambda t: (t.values[target_names[0]], t.values[target_names[1]])
                fig = optuna.visualization.plot_pareto_front(problem, target_names = target_names, targets = targets)
                fig.show()
            """
            """
            # Custom implementation
            import itertools
            objectives_combinations = list(itertools.combinations(list(range(len(objectives_data))), 2))
            for combination in objectives_combinations:
                objectives_history = []
                for i in range(n_objectives):
                    objectives = []
                    for trial in trials:
                        if trial.state == optuna.structs.TrialState.COMPLETE:
                            objectives.append(trial.values[i])
                    objectives_history.append(objectives)
                    
                plt.scatter(objectives_history[0], objectives_history[1], alpha=0.5)
                plt.xlabel(list(objectives_data.keys())[0])
                plt.ylabel(list(objectives_data.keys())[1])
            plt.show()
            """

        # Plot parameters history
        design_variables = config.get_design_variables()
        n_design_variables = len(design_variables)
        color = cm.rainbow(np.linspace(0, 1, n_design_variables))

        def order_of_mag(num):
            if num == 0:
                order = 0
            else:
                try:
                    order = math.floor(math.log10(num))
                except:
                    order = math.floor(math.log10(-num))
            return order
        all_vars_history = []
        min_order_of_mag = np.inf
        for i in range(n_design_variables):
            var_history = [trial.params[list(design_variables.keys())[i]] for trial in trials if trial.state == optuna.structs.TrialState.COMPLETE]
            all_vars_history.append(var_history)
            min_order_of_mag = order_of_mag(min(min_order_of_mag, min(var_history)))

        for i, c in zip(range(n_design_variables), color):
            var_scale_factor = 10 ** (order_of_mag(min(all_vars_history[i])) - min_order_of_mag)
            plotted_var_history = [all_vars_history[i][j] / var_scale_factor for j in range(len(all_vars_history[i]))]
            plt.scatter(t, plotted_var_history, c=c, label=f'{list(design_variables.keys())[i]} x {var_scale_factor}', alpha = 0.2)
            
#            moving_average = [sum(var_history[:i+1])/ (i+1) for i in range(len(var_history))]
#            plt.plot(t, moving_average, c=c, label="Moving avg for " + design_variables[i][0], alpha = 1.0, linewidth = 5)
#            
#            if n_objectives > 1:
    #            moving_average_per_iter = [sum(var_history[max(0, i- pop_size):i+1])/ (len(var_history[max(0, i- pop_size):i+1]) + 1) for i in range(len(var_history))]
#                moving_average_per_iter = [sum(var_history[max(0, (i // pop_size) * pop_size):i+1])/ (len(var_history[max(0, (i // pop_size) * pop_size):i+1]) + 1) for i in range(len(var_history))]
#                plt.plot(t, moving_average_per_iter, c=c, label="Moving avg per iter for " + design_variables[i][0], alpha = 1.0, linewidth = 5)
        plt.legend()
        plt.show()        
    
    def get_result_from_id(self, problem_name, storage_name, config):
        # Reload optimization problem
        problem = self.init_problem(problem_name, storage_name, config)

        # Ask user choice
        trials = problem.trials
        chosen_id = int(input("\nPlease pick the id of the chosen design between 0 and " + str(len(trials)) + ":"))
        return trials[chosen_id].params

    def get_best_results(self, problem_name, storage_name, config):
        # Reload optimization problem
        problem = self.init_problem(problem_name, storage_name, config)

        # Get best results
        objectives_data = config.get_objective_data()
        n_objectives = len(objectives_data)
        if n_objectives == 1:
            best_trial = problem.best_trial
        elif n_objectives > 1:
            best_trials = problem.best_trials
            print("Best objectives are:\n")
            best_objs = [best_trials[i].values for i in range(len(best_trials))]
            for num,item in enumerate(best_objs,start=0):
                print(num,item)
            choice_done = False
            chosen_id = int(input("Please pick the id of the chosen design:"))
            while choice_done == False:
                if 0 <= int(chosen_id) < len(best_trials):
                    choice_done = True
                else:
                    print("You should choose an integer between 0 and " + str(len(best_trials) - 1))
                    chosen_id = int(input("Please pick the id of the chosen design:"))
            best_trial = best_trials[chosen_id] 
        best_parameters = best_trial.params
        print("Selected best_params:", best_parameters)
        return best_parameters

    def init_problem(self, problem_name, storage_name, config):
        """
        Init optimziation problem.
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
        problem: optuna.study
            Study object from the optuna library
        """
        # Initialize sampler
        objectives_data = config.get_objective_data()
        n_objectives = len(objectives_data)
        sampler = self.init_sampler(n_objectives = n_objectives)
        objectives_data = config.get_objective_data()

        # Create/Load optimization problem     
        self.objective_directions = [objectives_data[obj][0] for obj in objectives_data]  
        problem = self.create_problem(problem_name, sampler, self.objective_directions, storage_name)

        return problem

    def init_sampler(self, n_objectives):
        """
        This function implements initialization of sampler.
        ----------
        Inputs
        ----------
        n_objectives: int
            Number of objectives.
        ----------
        Outputs
        ----------
        sampler: optuna.sampler
            Sampler object from the optuna library
        """
        # Choose a sampler depending on problem features
        if n_objectives == 1:
            if self.solver_name == "bayesian":
                sampler = optuna.samplers.TPESampler()
            elif self.solver_name == "evolutionary":
                sampler = optuna.samplers.CmaEsSampler(n_startup_trials = 20, restart_strategy = 'ipop', inc_popsize = 2)
        else:
            if self.solver_name == "bayesian":
                sampler = optuna.samplers.MOTPESampler(consider_prior=True)
            elif self.solver_name == "evolutionary":
                sampler = optuna.samplers.NSGAIISampler(population_size = 50, mutation_prob=None, crossover_prob=0.9, swapping_prob=0.5)   
        return sampler 

    def create_problem(self, problem_name, sampler, directions, storage_name):
        """
        This function implements initialization of an optimization problem.
        ----------
        Inputs
        ----------
        problem_name: str
            Name given to the optimization problem
        sampler: optuna.sampler
            Sampler object from the optuna library
        directions: list of str in {"minimize", "maximize"}
            Directions for objective optimization
        storage_name: str
            Path to the database where acquired data are stored.
        ----------
        Outputs
        ----------
        study: optuna.study
            Study object from the optuna library
        """
        # Reload a past study or create a new one
        optuna.logging.get_logger("optuna").addHandler(logging.StreamHandler(sys.stdout))
        try: 
            study = optuna.create_study(study_name=problem_name, sampler = sampler, directions=directions, storage=storage_name)         
        except: 
            study = optuna.create_study(study_name=problem_name, sampler = sampler, directions=directions, storage=storage_name, load_if_exists=True)     
        return study

    def sample_variables(self, trial, config):
        """
        Sample and update new values for design parameters based on previous tested variables
        ----------
        Inputs
        ----------
        trial: optuna.trial
            Class storing problem results
        config: Config
            Config class describing the optimization problem 
        ----------
        Outputs
        ----------
        updated_config: Config
            A copy of the config with updated sampled values for design variables
        """
        updated_config = config 
        variables = updated_config.get_design_variables()
        new_values = []
        for var_name in variables:
            new_values.append([var_name, trial.suggest_uniform(var_name, variables[var_name][1], variables[var_name][2])])     
        updated_config.set_design_variables(new_values)
        return updated_config
    
    def compute_objectives(self, trial, config):
        """
        Compute objectives for a set of sampled parameters
        ----------
        Inputs
        ----------
        trial: optuna.trial
            Class storing problem results
        config: Config
            Config class describing the optimization problem 
        """

        # Sample new design variables 
        updated_config = self.sample_variables(trial, config)
        
        try:
            # Generate geometry from sampled variables and simulate
            scores = self.evaluate_fitness(updated_config)  
        # Unfeasible design are penalized 
        except:
            print("[ERROR] >> The geometry is not properly generated.")
            scores = []
            # inf = 100000000000
            inf = np.inf
            for direction in self.objective_directions:
                if direction == "minimize":
                    scores.append(inf)
                elif direction == "maximize":
                    scores.append(- inf)
        return scores

        
