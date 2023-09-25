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

from Generation import Trunk, Corridor

class FitnessEvaluationController(BaseFitnessEvaluationController):   
    
    def __init__(self, *args, **kwargs):

        print('>>> Start Init SOFA scene ...')

        super(FitnessEvaluationController,self).__init__(*args, **kwargs)
        self.actuator = kwargs['actuator'] 
        self.goal_positions = kwargs["goal_positions"]

        # Objective evaluation variables
        self.current_iter = 0
        current_objectives = self.config.get_currently_assessed_objectives()
        
        # Init time counter
        self.current_iter = 0
        self.max_iter = max([self.config.get_objective_data()[current_objectives[i]][1] for i in range(len(current_objectives))])
        
        n_int_goals = len(self.goal_positions)
        self.goal_steps = int(self.max_iter / n_int_goals)
        self.current_goal_id = 0

    def onAnimateBeginEvent(self, dt):
        self.current_iter += 1

        #######################################
        ### Regularly update goal positions ###
        #######################################
        if self.current_goal_id+1 < len(self.goal_positions) - 1:
            if self.current_iter == (self.current_goal_id+1) * self.goal_steps:
                self.current_goal_id += 1 
                self.rootNode.Targets.dofs.position.value = self.goal_positions[self.current_goal_id+1]

        #################################################
        ### Compute objective when simulation is done ###
        #################################################
        if self.current_iter == self.max_iter:

            current_objectives_name = self.config.get_currently_assessed_objectives()   

            for i in range(len(current_objectives_name)):

                current_objective_name =  current_objectives_name[i]

                if "ShapeMatchingBigS" == current_objective_name:
                    self.ShapeMatchingMetric = self.rootNode.QPInverseProblemSolver.objective.value
                    print("Matching metric:", self.ShapeMatchingMetric)
                    self.objectives.append(self.ShapeMatchingMetric)

                if "Trajectory" == current_objective_name:
                    self.Trajectory = self.rootNode.QPInverseProblemSolver.objective.value
                    print("Matching metric:", self.Trajectory)
                    self.objectives.append(self.Trajectory)


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
    #rootNode.gravity = [0, 0, 0.]

    # General solver
    rootNode.addObject('FreeMotionAnimationLoop')
    if config.use_contact:
        rootNode.addObject('DefaultPipeline', name="CollisionPipeline")
        rootNode.addObject('BruteForceBroadPhase')
        rootNode.addObject('BVHNarrowPhase')
        rootNode.addObject('DefaultContactManager', name="CollisionResponse", response="FrictionContactConstraint", responseParams="mu=0.8")
        rootNode.addObject('LocalMinDistance', alarmDistance=0.8e-3, contactDistance=0.2e-3, angleCone=0.02)

    if config.inverse_mode:
        if config.use_contact:
            rootNode.addObject("QPInverseProblemSolver", 
                       epsilon = 1e-1, 
                       tolerance=1e-5, 
                    #    responseFriction = 0.8, 
                    #    allowSliding = False, 
                       minContactForces = 0,
                       # multithreading = True
                       ) 
        else:
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
    trunk_MO = trunk.addObject('MechanicalObject', name='dofs', template='Vec3')
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
    


    # Fix base of the Trunk
    trunk.addObject('BoxROI', name='boxROI', box=[[-20*config.mm, -20*config.mm, -config.d_ext-(config.d_in/2)], [20*config.mm, 20*config.mm, 0]], drawBoxes=True)
    #trunk.addObject('PartialFixedConstraint', fixedDirections=[1, 1, 1], indices='@boxROI.indices')
    trunk.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e10) 


    ##################
    ### Add camera ###
    ##################  
    if config.use_camera:
        pos_camera = [0, 0, config.total_length]
        camera_box = trunk.addObject('BoxROI', name='camera', 
                        box=[[pos_camera[0] - config.camera_dimensions[0]/2, pos_camera[1] - config.camera_dimensions[1]/2, pos_camera[2] - config.camera_dimensions[2]/2], 
                            [pos_camera[0] + config.camera_dimensions[0]/2, pos_camera[1] + config.camera_dimensions[1]/2, pos_camera[2] + config.camera_dimensions[2]/2]], 
                            drawBoxes=True)
        trunk.init() 
        
        all_indices = list(range(len(trunk_MO.findData('position').value)))
        remaining_indices = [index for index in all_indices if index not in camera_box.findData('indices').value]

        # Differentiate weights of both camera and body
        trunk_body = trunk.addChild("Body")
        trunk_body.addObject('UniformMass', indices = remaining_indices, 
                            totalMass= config.total_volume * config.volumetric_mass)
        
        camera = trunk.addChild("Camera")
        camera.addObject('UniformMass', indices = camera_box.findData('indices').value, 
                        totalMass = config.camera_weight)
    else: 
        trunk.addObject('UniformMass', totalMass= config.total_volume * config.volumetric_mass)



    ########################
    ### Collision Model ####
    ######################## 
    corridor_directions = ["x+y", "y+z", "x-y", "y-z",# Single corridor
                           "Ly+z", "Ly+z_120a", "Ly+z_240a", # L-shape
                           "Ty+z", #T-shape
                           ]
    corridor_plane =  corridor_directions[7] # Pick the direction of the obstacle
    gap, offset = 0.005, 0.005
    # Simple corridors
    if corridor_plane == "x+y":
        scaling_corridor = [[gap, 0.0600, 2/3 * config.total_length]] 
        translation_corridor = [[0.016, -scaling_corridor[0][1]/2, -offset]]
        rotation_matrix = np.array([[1, 0, 0],
                                    [0, 1, 0],
                                    [0, 0, 1]])

    elif corridor_plane == "y+z":
        scaling_corridor = [[0.0600, gap, 2/3 * config.total_length]] 
        translation_corridor = [[-scaling_corridor[0][0]/2, gap + config.r_in, -offset]]
        rotation_matrix = np.array([[1, 0, 0],
                                    [0, 1, 0],
                                    [0, 0, 1]])
    elif corridor_plane == "x-y":
        scaling_corridor = [[-gap, 0.0600, 2/3 * config.total_length]] 
        translation_corridor = [[- 0.016, -scaling_corridor[0][1]/2, -offset]]
        rotation_matrix = np.array([[1, 0, 0],
                                    [0, 1, 0],
                                    [0, 0, 1]])
    elif corridor_plane == "y-z":
        scaling_corridor = [[0.0600, -gap, 2/3 * config.total_length]] 
        translation_corridor = [[-scaling_corridor[0][0]/2, -gap - config.r_in, -offset]]
        rotation_matrix = np.array([[1, 0, 0],
                                    [0, 1, 0],
                                    [0, 0, 1]])

    ### L-shape and T-shape
    elif "L" or "T" in corridor_plane:

        # Defining the contact planes dimensions
        if "L" in corridor_plane:
            scaling_corridor = [
                                # Top left part
                                [0.0600, gap, 3/4 * config.total_length],
                                [0.0600, 3/4 * config.total_length, gap],
                                # Top right part
                                [0.0600, gap, 3/4 * config.total_length], 
                                [0.0600, 3/4 * config.total_length, gap],
                                # Bottom part
                                [0.0600, -gap, 2 * config.total_length]
                                ]
        
        elif "T" in corridor_plane:
            scaling_corridor = [
                                # Top left part
                                [0.0600, gap, 3/4 * config.total_length],
                                [0.5 * config.total_length, 3/4 * config.total_length, gap],
                                # Top right part
                                [0.0600, gap, 3/4 * config.total_length], 
                                [0.5 * config.total_length, 3/4 * config.total_length, gap],
                                # Bottom part
                                [0.0600, -gap, 2 * config.total_length],
                                # Knee in x-z plane
                                [0.5 * config.total_length, gap, 3/4 * config.total_length + gap],
                                [0.5 * config.total_length, gap, 3/4 * config.total_length + gap]
                                ]

        
        # Defining the contact planes location       
        if "L" in corridor_plane:           
            translation_corridor = [
                                    # Top left part
                                    [-scaling_corridor[0][0]/2, gap + config.r_in, -offset],
                                    [-scaling_corridor[0][0]/2, gap + config.r_in, 3/4 * config.total_length - offset],
                                    # Top right part
                                    [-scaling_corridor[0][0]/2, gap + config.r_in, config.total_length -offset],
                                    [-scaling_corridor[0][0]/2, gap + config.r_in, config.total_length - offset],
                                    # Bottom part
                                    [-scaling_corridor[0][0]/2, -gap - config.r_in, -offset]
                                    ]
        
        elif "T" in corridor_plane:
            translation_corridor = [
                                    # Top left part
                                    [-scaling_corridor[0][0]/2, gap + config.r_in, -offset - 1/2 * config.total_length],
                                    [-scaling_corridor[0][0]/2, gap + config.r_in, 1/2 * config.total_length - offset - 1/4 * config.total_length],
                                    # Top right part
                                    [-scaling_corridor[0][0]/2, gap + config.r_in, config.total_length -offset],
                                    [-scaling_corridor[0][0]/2, gap + config.r_in, config.total_length - offset],
                                    # Bottom part
                                    [-scaling_corridor[0][0]/2, -gap - config.r_in, -offset],
                                    # Knee in x-z plane
                                    [scaling_corridor[0][0]/2, gap + config.r_in, 1/4 * config.total_length - offset],
                                    [scaling_corridor[0][0]/2, gap + config.r_in + scaling_corridor[0][0]/2, 1/4 * config.total_length - offset],
                                    ]


        if "y+z" in corridor_plane:
            rotation_matrix = np.array([[1, 0, 0],
                                    [0, 1, 0],
                                    [0, 0, 1]])
        elif "y+z_120a" in corridor_plane:
            rotation_Z = 2.0944 # 120°
            rotation_matrix = np.array([[math.cos(rotation_Z), -math.sin(rotation_Z), 0],
                                    [math.sin(rotation_Z), math.cos(rotation_Z), 0],
                                    [0, 0, 1]])
        elif "y+z_240a" in corridor_plane:
            rotation_Z = 4.18879 # 240°
            rotation_matrix = np.array([[math.cos(rotation_Z), -math.sin(rotation_Z), 0],
                                    [math.sin(rotation_Z), math.cos(rotation_Z), 0],
                                    [0, 0, 1]])    
        


    if config.use_contact:
        # Trunk collision model
        contacts_trunk = trunk.addChild("CollisionModel")
        contacts_trunk.addObject('MeshSTLLoader', name='loader', filename= config.get_mesh_filename(mode = "Surface", refine = 1, 
                                                        generating_function = Trunk, 
                                                        n_modules = config.n_modules,
                                                        r_ext = config.r_ext, d_ext = config.d_ext, r_in = config.r_in, d_in = config.d_in,
                                                        min_radius_percent = config.min_radius_percent, min_module_size_percent = config.min_module_size_percent,
                                                        d_mspace = config.d_mspace, is_collision = True))
        contacts_trunk.addObject('TriangleSetTopologyContainer', src='@loader', name='container')
        contacts_trunk.addObject('MechanicalObject')
        # contacts_trunk.addObject('TriangleCollisionModel', group=1)
        # contacts_trunk.addObject('LineCollisionModel', group=1)
        contacts_trunk.addObject('PointCollisionModel', group=1)
        contacts_trunk.addObject('BarycentricMapping')

        # Add corridor
        for i in range(len(scaling_corridor)):
            corridor = simulation.addChild("Corridor_" + str(i))        
            corridor.addObject('MechanicalObject', template="Rigid3", scale="0.001", dx="0.0", dy="0.0", dz="0.0")
            #corridor.addObject("UniformMass", totalMass = 100.0)
            corridor.addObject('RestShapeSpringsForceField', stiffness=1e10)
            

            contacts_corridor = corridor.addChild("CollisionModel")
            # contacts_corridor.addObject('MeshSTLLoader', name='loader', filename = "Models/CabledTrunk/Meshes/Corridor/cube.stl",
            #                    scale3d = scaling_corridor, translation = translation_corridor)
            contacts_corridor.addObject('MeshSTLLoader', name='loader', filename = config.get_mesh_filename(mode = "Surface", refine = 2, 
                                                            generating_function = Corridor, x_scaling = scaling_corridor[i][0], 
                                                            y_scaling = scaling_corridor[i][1], z_scaling = scaling_corridor[i][2]),
                            translation = translation_corridor[i])
            contacts_corridor.addObject('TriangleSetTopologyContainer', src='@loader', name='container')
            corridor_MO = contacts_corridor.addObject('MechanicalObject', name='dofs', template='Vec3')
            contacts_corridor.init()
           
           # Rotate points
            rest_positions = list(corridor_MO.rest_position.value)
            for i in range(len(rest_positions)):
                position = rest_positions[i]
                rotated_position = np.dot(rotation_matrix, position)
                rest_positions[i] = rotated_position
            corridor_MO.position.value = rest_positions

            contacts_corridor.addObject('TriangleCollisionModel', group=2)
            # contacts_corridor.addObject('LineCollisionModel', group=2)
            # contacts_corridor.addObject('PointCollisionModel', group=2)
            contacts_corridor.addObject('RigidMapping')

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
                                maxPositiveDisp= 100e-3,
                                maxDispVariation = 10e-3,
                                minForce=0)
        cable.addObject('BarycentricMapping', name='mapping', mapForces=False, mapMasses=False)


    ### Add effector and goal if we are using an inverse model
    if config.inverse_mode:        
        
        current_objectives_name = config.get_currently_assessed_objectives() 

        # Compute Trunk Length
        config.update_total_length()
        trunk_length = config.total_length

        ########################################
        ### Manage shape matching objectives ###
        ########################################
        matching_objectives = ["ShapeMatchingBigS", "ShapeMatchingL", "ShapeMatchingS",
                               "ShapeMatchingCircularObject", "ShapeMatchingCubicObject"]
        if any(x in matching_objectives for x in current_objectives_name):
            # Sample effector and goal points for given matching scenario
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

        #####################################
        ### Manage exploration objectives ###
        #####################################
        if "Trajectory" in current_objectives_name:
            if config.use_contact:
                effector_positions = [[0, 0, trunk_length]]
                n_samples = 30
                if corridor_plane == "x+y":
                    goal_positions = compute_intermediate_points(
                        effector_positions[0],
                        [trunk_length / 3, 0, 2/3 * trunk_length],
                        n_samples)
                elif corridor_plane == "y+z":
                    goal_positions = compute_intermediate_points(
                        effector_positions[0],
                        [0, trunk_length / 3, 2/3 * trunk_length],
                        n_samples)
                elif corridor_plane == "x-y":
                    goal_positions = compute_intermediate_points(
                        effector_positions[0],
                        [-trunk_length / 3, 0, 2/3 * trunk_length],
                        n_samples)
                elif corridor_plane == "y-z":
                    goal_positions = compute_intermediate_points(
                        effector_positions[0],
                        [0, - trunk_length / 3, 2/3 * trunk_length],
                        n_samples)
                elif "L" in corridor_plane:
                    goal_positions = compute_intermediate_points(
                            effector_positions[0],
                            np.dot(rotation_matrix, [0, trunk_length / 4, 13/16 * trunk_length]),
                            n_samples)
                elif "T" in corridor_plane:
                    # Reaching the first milestone
                    goal_positions = compute_intermediate_points(
                            effector_positions[0],
                            np.dot(rotation_matrix, [0, trunk_length / 4, 5/16 * trunk_length]),
                            int(n_samples * 3/4))
                    # Toward the second milestone
                    goal_positions += compute_intermediate_points(
                            goal_positions[-1][0],
                            np.dot(rotation_matrix, [trunk_length / 4, trunk_length / 4, 5/16 * trunk_length]),
                            int(n_samples * 1/4))

            else:
                effector_positions = [[0, 0, 2/3 * trunk_length], [0, 0, trunk_length]]
                if corridor_plane == "x+y":
                    goal_positions = [[0, 0, 2/3 * trunk_length], [trunk_length / 3, 0, 2/3 * trunk_length]]
                elif corridor_plane == "y+z":
                    goal_positions = [[0, 0, 2/3 * trunk_length], [0, trunk_length / 3, 2/3 * trunk_length]]
                elif corridor_plane == "x-y":
                    goal_positions = [[0, 0, 2/3 * trunk_length], [-trunk_length / 3, 0, 2/3 * trunk_length]]
                elif corridor_plane == "y-z":
                    goal_positions = [[0, 0, 2/3 * trunk_length], [0, - trunk_length / 3, 2/3 * trunk_length]]

        # Goal
        target = rootNode.addChild('Targets')
        target.addObject('EulerImplicitSolver', firstOrder=True)
        target.addObject('CGLinearSolver', iterations=100, tolerance=1e-5, threshold=1e-5)
        target.addObject('MechanicalObject', name='dofs', position=goal_positions[0], showObject=True, showObjectScale=1.0*config.mm, drawMode=2, showColor = "red")
        target.addObject('UncoupledConstraintCorrection')

        # Effectors
        effectors = trunk.addChild('Effectors')
        effectors.addObject('MechanicalObject', position = effector_positions, showObject=True)
        effectors.addObject('PositionEffector', indices=list(range(len(effector_positions))), effectorGoal="@../../../Targets/dofs.position")
        effectors.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

    ##################
    ### Controller ###                            
    ##################
    rootNode.addObject(FitnessEvaluationController(name="FitnessEvaluationController", rootNode=rootNode, config=config, actuator=[], goal_positions = goal_positions))
    
    return rootNode


#############
### UTILS ###                            
#############
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


def compute_intermediate_points(init_point, end_point, n_points):
    """
    Interpolate for computing intermediate points between two coordinates.
    ----------
    Inputs
    ----------
    init_point: list of 3 floats
        Starting point of the trajectory the trajectory.
    end_point: list of 3 floats
        End point of the trajectory the trajectory.
    n_points: int
        Number of sample points to generate along the trajectory.
    ----------
    Outputs
    ----------
    points: list of list of list of 3 floats
        Sampled points describing along the trajectory.
    """
    intermediate_points = []
    for i in range(1, n_points + 1):
        alpha = i / (n_points + 1.0)  # Fraction between 0 and 1
        intermediate_point = [[
            init_point[0] + alpha * (end_point[0] - init_point[0]),
            init_point[1] + alpha * (end_point[1] - init_point[1]),
            init_point[2] + alpha * (end_point[2] - init_point[2])
        ]]
        intermediate_points.append(intermediate_point)
    return intermediate_points
