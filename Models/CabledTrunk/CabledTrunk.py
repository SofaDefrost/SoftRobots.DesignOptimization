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

                if "ShapeMatching" == current_objective_name:
                    self.objectives.append(0)



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
    rootNode.gravity = [0, -9.810, 0.]

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


    ### Add cables
    cables = trunk.addChild('cables')

    # Compute pull points location
    pull_points = []
    angle = 2 * math.pi / config.n_cables # Angle between two points on the circle
    for i in range(config.n_cables):
        pull_points.append([(config.r_in - config.dist_to_r_in) * math.cos(i * angle), (config.r_in - config.dist_to_r_in) * math.sin(i * angle), 0.])    

    # Create cables
    for i in range(config.n_cables):
        # Compute attachment points location
        positions = []
        dist_Z = 0
        for k in range(1, config.n_modules):
            r_scaling_factors = 1.0 - k * (1.0 - config.min_radius_percent) / config.n_modules
            d_scaling_factor = 1.0 - k * (1.0 - config.min_module_size_percent) / config.n_modules
            dist_Z += config.d_mspace + d_scaling_factor * (config.d_ext + config.d_in + config.d_ext)
            positions.append([(r_scaling_factors * config.r_in - config.dist_to_r_in) * math.cos(i * angle), 
                             (r_scaling_factors * config.r_in - config.dist_to_r_in) * math.sin(i * angle), 
                             dist_Z])
        
        print("positions:", positions)
        # Init cable
        cable = cables.addChild('cable_'+str(i))
        cable.addObject('MechanicalObject', name='dofs', position=positions)
        cable.addObject('CableConstraint' if not config.inverse_mode else 'CableActuator', template='Vec3', name='cable',
                                pullPoint = pull_points[i],
                                indices=list(range(0, config.n_modules - 1)),
                                maxPositiveDisp='70',
                                minForce=0)
        cable.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)


    ### Add effector and goal if we are using an inverse model
    if config.inverse_mode:
        target = rootNode.addChild('Targets')
        goal_positions = [[0., 0., 250*config.mm]]
        target.addObject('EulerImplicitSolver', firstOrder=True)
        target.addObject('CGLinearSolver')
        target.addObject('MechanicalObject', name='dofs', position=goal_positions, showObject=True, showObjectScale=8, drawMode=2, showColor=[1., 1., 1., 1.])
        target.addObject('UncoupledConstraintCorrection')

        effectors = trunk.addChild('Effectors')
        effector_positions = [[0., 0., config.Length]]
        effectors.addObject('MechanicalObject', position = effector_positions)
        effectors.addObject('PositionEffector', indices=list(range(len(effector_positions))), effectorGoal=target)
        effectors.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

    ##################
    ### Controller ###                            
    ##################
    rootNode.addObject(FitnessEvaluationController(name="FitnessEvaluationController", rootNode=rootNode, config=config, actuator=[]))
    
    return rootNode



