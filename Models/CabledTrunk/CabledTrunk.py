# -*- coding: utf-8 -*-
"""Config for the SensorFinger"""

__authors__ = "tnavez, qpeyron"
__contact__ = "tanguy.navez@inria.fr, quentin.peyron@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Feb 09 2023"

import importlib
import math
import numpy as np
import copy

import Sofa

from BaseFitnessEvaluationController import BaseFitnessEvaluationController

from Generation import Trunk

class FitnessEvaluationController(BaseFitnessEvaluationController):   
    
    def __init__(self, *args, **kwargs):

        print('>>> Start Init SOFA scene ...')

        super(FitnessEvaluationController,self).__init__(*args, **kwargs)
        self.actuator = kwargs['actuator'] 

        # Objective evaluation variables
        self.current_iter = 0
        current_objectives = self.config.get_currently_assessed_objectives()
        
        self.max_iter = max([self.config.get_objective_data()[current_objectives[i]][1] for i in range(len(current_objectives))])


    def onAnimateBeginEvent(self, dt):
        self.current_iter += 1

        if self.current_iter == self.max_iter:

            current_objectives_name = self.config.get_currently_assessed_objectives()   

            for i in range(len(current_objectives_name)):

                current_objective_name =  current_objectives_name[i]

                if "ShapeMatchingBigS" == current_objective_name:
                    self.ShapeMatchingMetric = self.rootNode.QPInverseProblemSolver.objective.value
                    print("Matching metric:", self.ShapeMatchingMetric)
                    self.objectives.append(self.ShapeMatchingMetric)



def createScene(rootNode, config):

    # Define the main architecture of the scene   
    rootNode.addObject('RequiredPlugin', pluginName= [
        'Sofa.Component.LinearSolver.Direct',
        'Sofa.Component.ODESolver.Backward',
        'Sofa.Component.Collision.Detection.Algorithm',
        'Sofa.Component.AnimationLoop',
        'Sofa.Component.Constraint.Lagrangian.Solver',
        'Sofa.GL.Component.Rendering3D',
        'Sofa.Component.SolidMechanics.FEM.Elastic',
        'Sofa.Component.SolidMechanics.Spring',
        'Sofa.Component.Engine.Select',
        'Sofa.Component.Mapping',
        'Sofa.Component.Collision.Detection.Intersection',
        'Sofa.Component.ODESolver.Backward Sofa.Component.IO.Mesh',
        'Sofa.Component.Mass',
        'Sofa.Component.Constraint.Lagrangian.Correction',
        'Sofa.Component.Constraint.Projective',
        'ArticulatedSystemPlugin'])
    rootNode.addObject("RequiredPlugin", name="SoftRobots")
    if config.inverse_mode:
        rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')
    
    # Setting the time step
    rootNode.dt = 0.01

    # Define the default view of the scene on SOFA
    rootNode.addChild('Settings')
    rootNode.Settings.addObject('OglSceneFrame')
    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.addObject('VisualStyle', displayFlags='hideInteractionForceFields showForceFields showCollisionModels showBehavior')

    # Weight
    #rootNode.gravity = [0, -9.810, 0.]
    rootNode.gravity = [0, 0, 0.]

    # General solver
    rootNode.addObject('FreeMotionAnimationLoop')
    if config.inverse_mode:
        rootNode.addObject('QPInverseProblemSolver', epsilon=1e-1)
    else:
        rootNode.addObject('GenericConstraintSolver', tolerance="1e-5", maxIterations="100")

    # Set up the pipeline for the mechanics computation
    simulation = rootNode.addChild('Simulation')
    simulation.addObject('EulerImplicitSolver', name='odesolver', firstOrder=False, rayleighMass=0.1, rayleighStiffness=0.1)
    simulation.addObject('SparseLDLSolver', name='precond')
    simulation.addObject('GenericConstraintCorrection')

    # Create the Trunk body
    trunk = simulation.addChild("Trunk")
    trunk.addObject('MeshVTKLoader', name='loader', filename = config.get_mesh_filename(mode = "Volume", refine = 1, 
                                                        generating_function = Trunk, 
                                                        n_modules = config.n_modules,
                                                        r_ext = config.r_ext, d_ext = config.d_ext, r_in = config.r_in, d_in = config.d_in,
                                                        min_radius_percent = config.min_radius_percent, min_module_size_percent = config.min_module_size_percent,
                                                        d_mspace = config.d_mspace))
    trunk.addObject('MeshTopology', src='@loader', name='container')
    trunk.addObject('MechanicalObject', name='dofs', template='Vec3')
    trunk.addObject('UniformMass', totalMass=0.1)
    trunk.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=0.45,  youngModulus=450000)


    # Add visual model 
    # trunk_visu = trunk.addChild('VisualModel')
    # trunk_visu.addObject('MeshSTLLoader', filename = config.get_mesh_filename(mode = "Surface", refine = 1, 
    #                                                     generating_function = Trunk,
    #                                                     n_modules = config.n_modules,
    #                                                     r_ext = config.r_ext, d_ext = config.d_ext, r_in = config.r_in, d_in = config.d_in,
    #                                                     min_radius_percent = config.min_radius_percent, min_module_size_percent = config.min_module_size_percent,
    #                                                     d_mspace = config.d_mspace))
    # trunk_visu.addObject('OglModel', color = [1., 1., 1., 1.])
    # trunk_visu.addObject('BarycentricMapping')

    # Add collision model
    # ??

    # Fix base of the Trunk
    trunk.addObject('BoxROI', name='boxROI', box=[[-20*config.mm, -20*config.mm, -config.d_ext-(config.d_in/2)], [20*config.mm, 20*config.mm, 0]], drawBoxes=True)
    #trunk.addObject('PartialFixedConstraint', fixedDirections=[1, 1, 1], indices='@boxROI.indices')
    trunk.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e10) 

    ##################
    ### Add cables ###
    ##################
    cables = trunk.addChild('cables')

    ### Compute pull points locations
    pull_points = []
    for c in range(config.n_cables + config.n_short_cables):
        angle = getattr(config, "theta_" + str(c) + "_0")
        pull_points.append([(config.r_in - config.dist_to_r_in) * math.cos(angle), (config.r_in - config.dist_to_r_in) * math.sin(angle), 0.])    

    # Create cables
    for c in range(config.n_cables + config.n_short_cables):
        # Compute attachment points location
        positions = []
        dist_Z = 0
        if c < config.n_cables:
            final_module = config.n_modules
        else:
            final_module = config.end_each_short_cable[c - config.n_cables]
        for m in range(1, final_module):
            angle = getattr(config, "theta_" + str(c) + "_" + str(m))
            r_scaling_factors = 1.0 - m * (1.0 - config.min_radius_percent) / config.n_modules
            d_scaling_factor = 1.0 - m * (1.0 - config.min_module_size_percent) / config.n_modules
            dist_Z += config.d_mspace + d_scaling_factor * (config.d_ext + config.d_in + config.d_ext)
            positions.append([(r_scaling_factors * config.r_in - config.dist_to_r_in) * math.cos(angle), 
                             (r_scaling_factors * config.r_in - config.dist_to_r_in) * math.sin(angle), 
                             dist_Z])
        
        # Init cable
        cable = cables.addChild('cable_'+str(c))
        cable.addObject('MechanicalObject', name='dofs', position=positions)
        cable.addObject('CableConstraint' if not config.inverse_mode else 'CableActuator', template='Vec3', name='cable',
                                pullPoint = pull_points[c],
                                indices=list(range(0, final_module - 1)),
                                # maxPositiveDisp='70',
                                minForce=0)
        cable.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)


    ### Add effector and goal if we are using an inverse model
    if config.inverse_mode:
        # Compute Trunk Length
        trunk_length = 0
        for k in range(config.n_modules):
            d_scaling_factor = 1.0 - k * (1.0 - config.min_module_size_percent) / config.n_modules
            if k == 0:
                trunk_length += d_scaling_factor * (config.d_ext + config.d_in / 2)
            else:
                trunk_length += d_scaling_factor * (config.d_ext + config.d_in + config.d_ext)
            if k != config.n_modules - 1:
                trunk_length += config.d_mspace

        # Sample effector and goal points for given matchign scenario
        current_objectives_name = config.get_currently_assessed_objectives()   
        if "ShapeMatchingBigS" in current_objectives_name:
            effector_positions, goal_positions = matching_scenario(name_scenario = "BigS", length = trunk_length, 
                                                n_samples = 20) 
        if "ShapeMatchingL" in current_objectives_name:
            effector_positions, goal_positions = matching_scenario(name_scenario = "L", length = trunk_length, 
                                           n_samples = 20) 
        if "ShapeMatchingS" in current_objectives_name:
            effector_positions, goal_positions = matching_scenario(name_scenario = "S", length = trunk_length, 
                                                n_samples = 20) 
        if "ShapeMatchingCircularObject" in current_objectives_name:
            effector_positions, goal_positions = matching_scenario(name_scenario = "CircularObject", length = trunk_length, 
                                                n_samples = 4) 
        if "ShapeMatchingCubicObject" in current_objectives_name:
            effector_positions, goal_positions = matching_scenario(name_scenario = "CubicObject", length = trunk_length, 
                                                n_samples = 12) 
        # effector_positions, goal_positions = matching_scenario(name_scenario = "basic", length = trunk_length, 
        #                                     n_samples = 1) 


        # Goal
        target = rootNode.addChild('Targets')
        target.addObject('EulerImplicitSolver', firstOrder=True)
        target.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
        target.addObject('MechanicalObject', name='dofs', position=goal_positions, showObject=True, showObjectScale=1.0*config.mm, drawMode=2, showColor = "red")
        target.addObject('UncoupledConstraintCorrection')

        # Effectors
        effectors = trunk.addChild('Effectors')
        effectors.addObject('MechanicalObject', position = effector_positions, showObject=True)
        effectors.addObject('PositionEffector', indices=list(range(len(effector_positions))), effectorGoal="@../../../Targets/dofs.position")
        effectors.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

    ##################
    ### Controller ###                            
    ##################
    rootNode.addObject(FitnessEvaluationController(name="FitnessEvaluationController", rootNode=rootNode, config=config, actuator=[]))
    
    return rootNode



def matching_scenario(name_scenario, length, n_samples):
    """
    Generate both effector and target points needed for a given matchign scenario.
    ----------
    Inputs
    ----------
    name_scenario: string 
        Name of the matching scenario.
    length: float
        2D Length of the shape to generate.
    n_samples: int
        Number of sample points to generate on the shape.
    ----------
    Outputs
    ----------
    effector_points: list of list of 3 floats
        Points on the Trunk, to match with target points.
    target_points: list of list of 3 floats
        Target points describing the input shape.
    """

    if name_scenario == "basic":
        effector_points = [[0., 0., length]]
        target_points = [[0., -10.0e-3, length]]
    elif name_scenario == "S":
        length = 2 * length / 3
        translationZ = 0.5 * length / 3 
        effector_points = [[0., 0., translationZ + i*length/(n_samples-1)] for i in range(n_samples)]
        target_points = generate_shape("BigS", length, n_samples)
        for i in range(len(target_points)):
            target_points[i][2] += translationZ
    elif name_scenario == "BigS":
        effector_points = [[0., 0., i*length/(n_samples-1)] for i in range(n_samples)]
        target_points = generate_shape("BigS", length, n_samples)
    elif name_scenario == "L":
        effector_points = [[0., 0., i*length/(n_samples-1)] for i in range(n_samples)]
        target_points = generate_shape("L", length, n_samples)
    elif name_scenario == "CircularObject":
        length = 2 * length / 3
        translationZ = 2.0 * length / 3 
        half_circle_perimeter = 2 * np.pi * 25e-3 / 2
        effector_points = [[0., 0., translationZ + i*half_circle_perimeter/(n_samples-1)] for i in range(n_samples)]
        target_points = generate_shape("CircularObject", length, n_samples)
    elif name_scenario == "CubicObject":
        length = 2 * length / 3
        translationZ = 2.0 * length / 3 
        half_cube_perimeter = 2 * np.pi * 25e-3 / 2
        effector_points = [[0., 0., translationZ + i*half_cube_perimeter/(n_samples-1)] for i in range(n_samples)]
        target_points = generate_shape("CubicObject", length, n_samples)
    return effector_points, target_points


def generate_shape(name_shape, length, n_samples):
    """
    Sample n points describing a given shape with given length.
    ----------
    Inputs
    ----------
    name_shape: string in {S, ...}
        Name of the shape to generate.
    length: float
        2D Length of the shape to generate.
    n_samples: int
        Number of sample points to generate on the shape.
    ----------
    Outputs
    ----------
    points: list of lsit of 3 floats
        Sampled points describing the input shape.
    """

    if name_shape == "BigS":
        # S shape
        t = np.linspace(-np.pi/2, np.pi/2, n_samples)  
        y = (length/2) * np.sin(2*t) / 2  
        z = length/2 + (length/2) * np.sin(t) 
        # Generate 3D points
        points = [[0., y[i], z[i]] for i in range(n_samples)]
    
    elif name_shape == "L":
        points = []
        step = length / n_samples
        for i in range(2 * n_samples // 3):
            points.append([0., 0., i*step])
        for i in range(0, n_samples // 3):
            points.append([0., i*step, (2 * n_samples // 3) * step])
    
    elif name_shape == "CircularObject":
        points = []
        center = [40e-3, 0, 65e-3]
        radius = 25e-3
        for i in range(n_samples):
            angle_step = (1/2) * 2 * math.pi * i / n_samples
            x = center[0] - radius * math.cos(angle_step)
            y = center[1] 
            z = center[2] + radius * math.sin(angle_step)
            points.append([x,y,z])

    elif name_shape == "CubicObject":
        points = []
        center = [40e-3, 0, 65e-3]
        side_length = 25e-3
        step =  3 *side_length / n_samples
        # Generate points on bottom part of square
        for i in range(int(n_samples / 3)):
            x = center[0] - side_length / 2 
            y = center[1]
            z = center[2] - side_length / 2 + i * step
            points.append([x,y,z])

        # Generate points on right part of square
        for i in range(int(n_samples / 3)):
            x = center[0] - side_length / 2 + i * step
            y = center[1]
            z = center[2] + side_length / 2 
            points.append([x,y,z])

        # Generate points on upper part of square
        for i in range(int(n_samples / 3)):
            x = center[0] + side_length / 2 
            y = center[1]
            z = center[2] + side_length / 2 - (i+1) * step
            points.append([x,y,z])



    return points