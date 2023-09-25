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
import pyvista
import copy
import csv

import Sofa

# from splib3.objectmodel import SofaPrefab
# from stlib3.scene import Scene
# from stlib3.scene.contactheader import ContactHeader

fixing_box_lib = importlib.import_module("Components.fixing_box")
cylinder_lib = importlib.import_module("Components.cylinder")
actuated_finger_lib = importlib.import_module("Components.actuated_finger")

from BaseFitnessEvaluationController import BaseFitnessEvaluationController

from Generation import TripodFinger, ContactSurfaceIn1, ContactSurfaceIn2, ContactSurfaceOut


class FitnessEvaluationController(BaseFitnessEvaluationController):   
    
    def __init__(self, *args, **kwargs):

        print('>>> Start Init SOFA scene ...')

        super(FitnessEvaluationController,self).__init__(*args, **kwargs)
        self.actuator = kwargs['actuator']
        self.ModelNode = self.rootNode.Modelling     

        # Link to collision models between soft finger and object
        self.contact_finger_collis = self.rootNode.Modelling.ActuatedFinger.ElasticBody.ElasticMaterialObject.CollisionMeshOut.getMechanicalState()
        if self.config.use_object:
            self.contact_cylinder_collis = self.ModelNode.Obstacle.Cylinder.collision.getMechanicalState()

        # Computation of the contact force applied on the object to grasp
        self.rootNode.getRoot().GenericConstraintSolver.computeConstraintForces.value = True
        self.max_angle = self.config.max_angle
        self.angleInit = 0
        
        # Objective evaluation variables
        self.current_iter = 0
        current_objectives = self.config.get_currently_assessed_objectives()
        self.max_iter = max([self.config.get_objective_data()[current_objectives[i]][1] for i in range(len(current_objectives))])

        # Time and intermediate angle intervals
        self.iter_eval = self.max_iter - 5
        self.n_target_angles = self.config.n_target_angles # Number of intermediate angles
        self.break_iter_per_step = 3 # Number of iter for reaching equilibrium between each intemediate target angle, for avoiding dynamical effect      
           
        self.init_angle_targets()

        # Evaluation and Calibration data lists
        self.list_force_X = []
        self.int_interval_torques = []
        self.list_data = [[]]
        

    def init_angle_targets(self):
        """"
        This method create a list with all target angles to reach.
        """
        # Set the maximum number of iterations to be a multiple of the number of intermediate targets
        self.iter_eval = self.iter_eval - self.iter_eval % self.n_target_angles

        # Compute angular steps
        angle_step = self.max_angle / (self.iter_eval - self.n_target_angles * self.break_iter_per_step) # We do not count the break iter

        # Compute the list of angles to reach
        self.angles_per_dt = []
        self.target_angles = []
        self.final_break_dt = []
        prev_angle = self.angleInit
        increments_per_target = int(self.iter_eval / self.n_target_angles - self.break_iter_per_step)
        for t in range(self.n_target_angles):
            # Add intermediate angles to reach target
            self.angles_per_dt += [prev_angle + (a+1) * angle_step 
                                   for a in range(increments_per_target)]
            prev_angle = self.angles_per_dt[-1]
            self.target_angles.append(prev_angle)
            # Add breaks
            self.angles_per_dt += [prev_angle for a in range(self.break_iter_per_step)]
            self.final_break_dt.append((t+1) * (increments_per_target + self.break_iter_per_step) - 1)
        print("Number of target angles:", len(self.target_angles))
        print("Target angles:", str(self.target_angles))

    # A method for dealing with one constraint
    def _dealConstraint(self, s):
        """"
        -------
        Input:
        -------
        s: SOFA constraint
        Link to the considered constraint state
        -------
        Outputs:
        -------
        A dictionnary describing the constraint with the following attributes:
        id: int
            Id of the constraint
        nb_points: int
            Number of points implied in the constraint
        points: list of int
            Indices of points (mechanical object related to s) implied in the constraint.
        """
        if s!='':
            inter_s = s.split(' ')

            id_constraint = int(inter_s[0])
            num_points = int(inter_s[1])

            points = []
            for i in range(num_points):
                point = int(inter_s[2+5*i])
                points.append(point)

            return {'id': id_constraint, 'nb_point': num_points, 'points': points}
        else:
            return {'id': None}
    
    # A method for dealing with several constraints
    def _dealConstraints(self, s, particular_point = None):
        inter_s = s.split("\n")
        idx = set()
        correspondance = {}
        for constraint in inter_s:
            cons = self._dealConstraint(constraint)
            if particular_point is not None and cons['id'] is not None:
                if particular_point in cons["points"]:
                    id = cons['id']
                else:
                    id = None
            else:
                id = cons['id']
            if id is not None:
                idx.add(id)
                correspondance.update({id: cons["points"]})
        return idx, correspondance


    def onAnimateBeginEvent(self, dt):
        MAX_SERVO = 1.2 # Herkulex limit in N.m.

        angle = 0.0
        ### Incremental update of servomotor angular position
        if self.current_iter < self.iter_eval:
            angle = self.angles_per_dt[self.current_iter]
        else:
            angle = self.angleInit + self.max_angle

        self.actuator.ServoMotor.angleIn = angle

        ### Initial coordinates of collision meshes vertices
        if self.current_iter == 0:
            self.coords_finger_collis_init = copy.deepcopy(self.contact_finger_collis.position.value)
            if self.config.use_object:
                self.coords_object_collis_init = copy.deepcopy(self.contact_cylinder_collis.position.value)

        ### Register both current forces and current torque for evaluation/calibration purpose
        if len(self.final_break_dt) != 0:
            if self.current_iter == self.final_break_dt[0]:
                self.final_break_dt.pop(0)
                # Torque data
                self.int_interval_torques.append(self.evaluate_torque().tolist()[0])
                #print("self.int_interval_torques:", self.int_interval_torques)
                # Contact Force data
                if self.config.use_object:
                    self.evaluate_forces()
                    self.list_force_X.append(self.forceContactX)
                    # print("self.list_force_X:", self.list_force_X)

        ### Save torque and angular displacement at each time step
        self.list_data.append([angle,self.evaluate_torque()[0][0]])

        ### Display when reaching herkulex limite
        if len(self.int_interval_torques) != 0:
            if abs(self.int_interval_torques[-1][0]) >  MAX_SERVO:
                print("Reached servo limit at angular step", len(self.int_interval_torques))
            # print("Actual couple:", self.int_interval_torques[-1][0])

        self.current_iter += 1
        
        ### Evaluated forces applied on the cylinder
        if self.current_iter == self.max_iter:
            
            current_objectives_name = self.config.get_currently_assessed_objectives()   

            # As the torque is modeled as a Spring, we compute the torque as k * (theta - theta_0)
            self.MotorTorque = self.evaluate_torque()

            # Compute contact forces between the finger and the object
            if self.config.use_object:
                self.evaluate_forces()

            for i in range(len(current_objectives_name)):

                current_objective_name =  current_objectives_name[i]

                # Penalise interpenetration of soft finger with object in the scene at beginning of simulation
                if "InterpenetrationPenaltyInitX" == current_objective_name:
                    # Penalise interpenetrating objects
                    self.interpenetrationPenaltyInitX = 0
                    is_interpenetrating = self.check_interpenetrationX(self.coords_finger_collis_init, self.coords_object_collis_init)
                    if is_interpenetrating:
                        self.interpenetrationPenaltyInitX = 1000
                    print("Interpenetration Init Penalty along X-axis:", self.interpenetrationPenaltyInitX)
                    self.objectives.append(self.interpenetrationPenaltyInitX)

                # Mass of material needed for building the Tripod Finger
                if "Mass" == current_objective_name:
                    volumeMeshFileName = self.config.get_mesh_filename(mode = "Surface", refine = 0, 
                                                        generating_function = TripodFinger,
                                                        L = self.config.L, l = self.config.l, e1 = self.config.e1, 
                                                        e2 = self.config.e2, e3 = self.config.e3, n = self.config.n, 
                                                        d = [getattr(self.config, 'd'+ str(i)) for i in range(self.config.n)], 
                                                        w = self.config.w, lc = self.config.lc)
                    mesh = pyvista.read(volumeMeshFileName)
                    self.mass = self.config.rho*mesh.volume
                    print("Mass of the Finger: ", self.mass)
                    self.objectives.append(self.mass)

                # Contact Force along X-axis
                if "ContactForceX" == current_objective_name:
                    # print("The Contact Force along X: "+str(self.forceContactX))
                    # self.objectives.append(self.forceContactX)

                    acceptable_forces = [abs(self.list_force_X[i])
                                            for i in range(len(self.int_interval_torques))
                                            if self.list_force_X[i] != 0 # Remove case where there is no contact 
                                            and abs(self.int_interval_torques[i][0]) <=  MAX_SERVO] # Remove unreachable state because of hardware limitations
                    if len(acceptable_forces) == 0:
                        self.BestContactForceXObjective = 100000
                    else:
                       self.BestContactForceXObjective = max(acceptable_forces) 

                    print("All met contact forces, start:\n")
                    print(*acceptable_forces, sep="\n")
                    print("... end")

                    print("The Contact Force along X: "+str(self.BestContactForceXObjective))
                    self.objectives.append(self.BestContactForceXObjective)


                # Norm of the Contact Force
                if "ContactForceNorm" == current_objective_name:
                    self.forceContactN = (self.forceX**2+self.forceY**2+self.forceZ**2)**0.5 
                    print("Norm of the Contact Force: "+str(self.forceContactN))
                    self.objectives.append(self.forceContactN)

                # Force Transmission along X-axis
                # It is the ratio between the necessary Motor Torque and the resulting gripping force along X-axis 
                if "ForceTransmissionX" == current_objective_name:
                    self.forceTransmissionX = abs(self.MotorTorque) / abs(self.forceContactX)  
                    print("Torque="+str(self.MotorTorque))
                    print("Contact Force along X-axis: " + str(self.forceContactX))
                    print("Force Transmission along X-axis: "+ str(self.forceTransmissionX))
                    self.objectives.append(self.forceTransmissionX)

                # Best Force Transmission along X-axis evaluated for several angular positions
                # It is the best ratio between the necessary Motor Torque and the resulting gripping force along X-axis 
                # It is evaluated under the constraint of maximum force exerted by the servomotor.
                if "IncrementalForceTransmissionX" == current_objective_name:
                    acceptable_transmissions = [abs(self.list_force_X[i]) / abs(self.int_interval_torques[i][0])
                                                      for i in range(len(self.int_interval_torques))
                                                      if self.list_force_X[i] != 0 # Remove case where there is no contact 
                                                      and abs(self.int_interval_torques[i][0]) <=  MAX_SERVO] # Remove unreachable state because of hardware limitations
                    if len(acceptable_transmissions) == 0:
                        self.BestForceTransmissionXObjective = 1
                    else:
                        self.BestForceTransmissionXObjective = 1 / max(acceptable_transmissions) 


                    print("All torques, start:\n")
                    print(*[abs(self.int_interval_torques[i][0]) for i in range(len(self.int_interval_torques))], sep="\n")
                    print("... end")

                    print("Force transmissions (force / actuation) while in contact, start:\n")
                    print(*acceptable_transmissions, sep="\n")
                    print("... end")

                    print("Best Force Transmission (force / actuation):", max(acceptable_transmissions))
                    print("Objective Force Transmission (actuation / force) along X-axis: "+ str(self.BestForceTransmissionXObjective))
                    self.objectives.append(self.BestForceTransmissionXObjective)

                # Metric for evaluating the energy needed for performing a grasp
                # The total electric energy is directly proportional to the generated couples on the finger 
                # in our setup as we sample both with fixed angle and time steps
                if "GraspingEnergy" == current_objective_name:
                    mechanical_works = [abs(self.int_interval_torques[i][0]) * abs(self.target_angles[i])  
                                                      for i in range(len(self.int_interval_torques))
                                                      if abs(self.int_interval_torques[i][0]) <=  MAX_SERVO] # Remove unreachable state because of hardware limitations
                    if len(mechanical_works) == 0:
                        self.GraspingEnergy = 0
                    else:
                        self.GraspingEnergy = sum(mechanical_works)

                    print("Objective Grasping Energy: ", self.GraspingEnergy)
                    self.objectives.append(self.GraspingEnergy)


                # Penalise interpenetration of soft finger with object in the scene at end of simulation
                if "InterpenetrationPenaltyEndX" == current_objective_name:
                    # Get contact positions
                    coords_finger_collis = self.contact_finger_collis.position.value
                    coords_object_collis = self.contact_cylinder_collis.position.value

                    # Penalise interpenetrating objects
                    self.interpenetrationPenaltyEndX = 0
                    is_interpenetrating = self.check_interpenetrationX(coords_finger_collis, coords_object_collis)
                    if is_interpenetrating:
                        self.interpenetrationPenaltyEndX = 1000
                    print("Interpenetration End Penalty along X-axis:", self.interpenetrationPenaltyEndX)
                    self.objectives.append(self.interpenetrationPenaltyEndX)

                # Priorize contact force focused in one point for precision grasping.
                if "LocateContactPrecision" == current_objective_name:
                    TARGET = np.array(self.rootNode.Modelling.Obstacle.ContactLocation.dofs.position.value[0]) # Coordinates of the point where the contact force should be situated
                    THRESHOLD = 1e-3 # Area around the TARGET point where we want contacts

                    # Get constraints data concerning each collision mesh
                    idx_constraint_finger_collis, idx_points_finger_collis =  self._dealConstraints(self.contact_finger_collis.constraint.value)
                    idx_constraint_object_collis, idx_points_object_collis =  self._dealConstraints(self.contact_cylinder_collis.constraint.value)
                    
                    # Find collision constraints between finger and object
                    idx_collision_finger_object = self.match_constraint_lists(idx_constraint_finger_collis, idx_constraint_object_collis)

                    # Retrieve the coordinates of the collision points
                    coord_object_collis = {}
                    contact_cylinder_collis_MO = self.contact_cylinder_collis.position.value
                    for id in idx_collision_finger_object:
                        coord_object_collis[id] = contact_cylinder_collis_MO[idx_points_object_collis[id]]

                    ### Penalise collision points weighted by force that are far away from center point
                    # Find coordinates of collision points on object for each constraint between finger and object
                    collision_coordinates = {}
                    for id in idx_collision_finger_object:
                        avg_coord = [sum(x)/len(x) for x in zip(*coord_object_collis[id])]
                        collision_coordinates[id] = avg_coord

                    # Compute location penalty weighted by force
                    self.contactForceXLocationPenalty = 0
                    sum_forceX = 0
                    for id in idx_collision_finger_object:
                        norm = (self.constraints_data[id][0]**2+self.constraints_data[id][1]**2+self.constraints_data[id][2]**2)**0.5 
                        curr_forceX = abs(1/norm*self.contactForces[id]*self.constraints_data[id][0]) / self.rootNode.dt.value
                        self.contactForceXLocationPenalty += (abs(TARGET[2] - collision_coordinates[id][2]) - THRESHOLD) * curr_forceX
                        sum_forceX += curr_forceX

                    # Normalize by sum of forces
                    # self.contactForceXLocationPenalty = self.contactForceXLocationPenalty / sum_forceX

                    print("Contact Location Penalty along x-axis: "
                        + str(self.contactForceXLocationPenalty))
                    self.objectives.append(self.contactForceXLocationPenalty)

                # Calibration metric using torque
                if "TorqueCalibration" == current_objective_name:

                    # List of simulated torques, taking into account an eventual offset initTorque
                    sim_torque = np.reshape(self.int_interval_torques, (1, self.n_target_angles))[0]-self.config.initTorque*np.ones(self.n_target_angles)
                    print("Torques measured in simulation", sim_torque)

                    # List of measured torque on the physical prototype
                    if self.config.use_object:
                        # Experimental data: centerline of the pointcloud, with the offset, for N = 8
                        ground_truth_torques = [-0.1117, -0.1997, -0.1940, -0.3172, -0.3711, -0.5734, -0.8411, -1.6110]
                    else: 
                        # ground_truth_torques = [-0.04, -0.07, -0.10, -0.15, -0.18]
                        # Experimental data: simple interpolation
                        ground_truth_torques = [-0.17, -0.18, -0.2829, -0.325, -0.44]
                        # Experimental data: linear regression and interpolation, without the offset
                        ground_truth_torques = [-0.076, -0.152, -0.228, -0.304, -0.380]

                    error = np.subtract(np.array(ground_truth_torques),sim_torque)
                    print("Errors: " + str(error))

                    # RMS value of the error as objective function to minimize
                    self.TorqueCalibrationMetric = np.sqrt(np.mean(error**2))

                    # Saves the values of angular displacement and torque during the complete simulation
                    # with open('TripodFinger_angle_torque_Eopt_0p02176_objet30.csv', 'w', newline='') as f:
                    #     # using csv.writer method from CSV package
                    #     write = csv.writer(f, delimiter=',',
                    #                        quotechar='|', quoting=csv.QUOTE_MINIMAL)
                    #     for k in range(0, len(self.list_data)):
                    #         write.writerow(self.list_data[k])

                    print("Torque calibration metric:" + str(self.TorqueCalibrationMetric))
                    self.objectives.append(self.TorqueCalibrationMetric)
                    

                # Calibration metric using contact force
                if "ContactForceCalibration" == current_objective_name:
                    # List of simulated contact forces in g
                    sim_forces = np.array([self.list_force_X[i] * 1000 / 9.81 for i in range(len(self.list_force_X))])
                    print("Contact forces (in g) measured in simulation", sim_forces)

                    # Forces measured on physical prototype
                    ground_truth_forces = np.array([0, 0, 0, 0, 0, 8.8, 57.6, 106.4, 155.2, 204])
                    print("Contact forces (in g) measured on real prototype", ground_truth_forces)
                    error = np.subtract(ground_truth_forces, sim_forces)
                    print("Element wise error for cotnact force calibration:", error)
                    self.ContactForceCalibrationMetric = np.sqrt(np.mean(error**2))
                    print("Contact force calibration metric: " + str(self.ContactForceCalibrationMetric))
                    self.objectives.append(self.ContactForceCalibrationMetric)



    def evaluate_torque(self):
        # As the torque is modeled as a Spring, we compute the torque as k * (theta - theta_0)
        k = self.actuator.ServoMotor.Articulation.RestShapeSpringsForceField.stiffness.value
        theta_0 = self.actuator.ServoMotor.Articulation.dofs.rest_position.value 
        theta = self.actuator.ServoMotor.Articulation.dofs.position.value
        return k * (theta - theta_0)
    
    def evaluate_forces(self):
        # Compute the force exerted on an object
        self.contactForces = self.rootNode.getRoot().GenericConstraintSolver.constraintForces.value
        constraint= self.rootNode.Modelling.Obstacle.Cylinder.collision.MechanicalObject.constraint.value.split('\n')[0:-1]
        indices_constraint=[]
        self.numContact=len(constraint)
        self.forceContactX=0
        self.forceX,self.forceY,self.forceZ=0,0,0
        # print("Start evaluating forces ...")
        for i in range(len(constraint)):
            indices_constraint.append([int(constraint[i].split(' ')[0]),float(constraint[i].split(' ')[3]),float(constraint[i].split(' ')[4]),float(constraint[i].split(' ')[5])])
            # print("indices_constraint[i]:", indices_constraint[i])
            # print("self.contactForces[indices_constraint[i][0]]:", self.contactForces[indices_constraint[i][0]])
            norm=(indices_constraint[i][1]**2+indices_constraint[i][2]**2+indices_constraint[i][3]**2)**0.5
            ###### Evaluate the normal force
            self.forceContactX += 1/norm*self.contactForces[indices_constraint[i][0]]*indices_constraint[i][1] 
            # print("Additional force X:", 1/norm*self.contactForces[indices_constraint[i][0]]*indices_constraint[i][1] )
            # print("self.forceContactX:", self.forceContactX)
            ####### Evaluate force norm
            self.forceX+=1/norm*self.contactForces[indices_constraint[i][0]]*indices_constraint[i][1]
            self.forceY+=1/norm*self.contactForces[indices_constraint[i][0]]*indices_constraint[i][2]
            self.forceZ+=1/norm*self.contactForces[indices_constraint[i][0]]*indices_constraint[i][3]
        # print("... end")
        # In a previous version of SOFA, force*dt was stored in lambda vectors for ease of use. 
        # This piece of code was necessary to remove the dt part for obtaining only the force
        # self.forceContactX /= self.rootNode.dt.value
        # self.forceX /= self.rootNode.dt.value 
        # self.forceY /= self.rootNode.dt.value
        # self.forceZ /= self.rootNode.dt.value

        # Contact constraints are counted in double

        # Build dict of constraint data from solver
        self.constraints_data = {}
        for constraint in indices_constraint:
            self.constraints_data[constraint[0]] = constraint[1:]
    
    def check_interpenetrationX(self, coords_0, coords_1):
        """
        Check in 2D if two meshes are interpenetrating along x-axis.
        ----------
        Inputs
        ----------
        coords_0: list of list of 3 floats
            Coordinates of the first mesh vertices.
        coords_1: list of list of 3 floats
            Coordinates of the second mesh vertices to not outpass along x-axis.
        ----------
        Outputs
        ----------
        is_interpentrating: boolean
            Indicate if the two meshes are interpenetrating.
        """
        # Get object limit on X-axis
        object_start_x = min(coords_1, key=lambda c: c[0])[0]
        
        # Penalise interpenetrating objects
        epsilon = 0.001
        for coord in coords_0:
            if coord[0] > object_start_x + epsilon:
                return True                   
        return False


    def match_constraint_lists(self, idx_0, idx_1):
        """
        Compute indices of common constraints between two SOFA entities.
        ----------
        Inputs
        ----------
        idx_0: list of int
            Constraint indices of first SOFA entity.
        idx_1: list of int
            Constraint indices of second SOFA entity.
        ----------
        Outputs
        ----------
        matching_ids: list of int
            List of common constraint ids
        """
        matching_ids = []
        for id in idx_0:
            if id in idx_1:
                matching_ids.append(id)
        return matching_ids


def createScene(rootNode, config):
    
    # Define the main architecture of the scene, with a node Modelling, Setting and Simulation
    rootNode.addChild('Modelling')
    rootNode.addChild('Settings')
    rootNode.addChild('Simulation')    
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
    rootNode.Settings.addObject('OglSceneFrame')
    rootNode.findData('gravity').value = [0, 0, -9.810] 

    # Setting the time step
    rootNode.dt = 0.01

    # Define the default view of the scene on SOFA
    rootNode.addObject('DefaultVisualManagerLoop')
    rootNode.addObject('VisualStyle', displayFlags='hideInteractionForceFields showForceFields showCollisionModels')
    rootNode.addObject("VisualGrid", nbSubdiv=100, size=1)

    # Solvers
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultPipeline')
    rootNode.addObject('BruteForceBroadPhase')
    rootNode.addObject('BVHNarrowPhase')
    frictionCoef= 0.85 # 0.85
    rootNode.addObject('RuleBasedContactManager', responseParams="mu="+str(frictionCoef),
                                                    name='Response', response='FrictionContactConstraint')#frictionCoef=0.1
    rootNode.addObject('LocalMinDistance',alarmDistance=5e-3, contactDistance=0.5e-3, angleCone=0.01)
    
    # Set up the pipeline for the collision and mechanics computation
    rootNode.addObject('GenericConstraintSolver', tolerance="1e-6", maxIterations="10000")
    rootNode.Simulation.addObject('GenericConstraintCorrection')
    rootNode.Simulation.addObject('EulerImplicitSolver', name='TimeIntegrationSchema',firstOrder = True)
    rootNode.Simulation.addObject('SparseLDLSolver', name='LinearSolver', template="CompressedRowSparseMatrixMat3x3d")

    # Create one actuated finger
    actuatedFinger = actuated_finger_lib.ActuatedFinger(
        youngModulus = config.youngModulus * 1.0e9, # In Pa
        poissonRatio = config.poissonRatio,
        stlMeshFileNameIn1 = config.get_mesh_filename(mode = "Surface", refine = 0, 
                                                        generating_function = ContactSurfaceIn1,
                                                        L = config.L, e1 = config.e1, e2 = config.e2, 
                                                        w = config.w, lc = config.lc), 
        stlMeshFileNameIn2 = config.get_mesh_filename(mode = "Surface", refine = 0, 
                                                        generating_function = ContactSurfaceIn2,
                                                        L = config.L, l = config.l, e1 = config.e1, 
                                                        e2 = config.e2, e3 = config.e3, n = config.n, 
                                                        d = [getattr(config, 'd'+ str(i)) for i in range(config.n)], 
                                                        w = config.w, lc = config.lc), 
        stlMeshFileNameOut = config.get_mesh_filename(mode = "Surface", refine = 0, 
                                                        generating_function = ContactSurfaceOut,
                                                        L = config.L, l = config.l, e1 = config.e1, 
                                                        e2 = config.e2, e3 = config.e3, n = config.n, 
                                                        d = [getattr(config, 'd'+ str(i)) for i in range(config.n)], 
                                                        w = config.w, lc = config.lc),
        stlMeshFileName = config.get_mesh_filename(mode = "Surface", refine = 0, 
                                                        generating_function = TripodFinger,
                                                        L = config.L, l = config.l, e1 = config.e1, 
                                                        e2 = config.e2, e3 = config.e3, n = config.n, 
                                                        d = [getattr(config, 'd'+ str(i)) for i in range(config.n)], 
                                                        w = config.w, lc = config.lc), 
        volumeMeshFileName = config.get_mesh_filename(mode = "Volume", refine = 0, 
                                                        generating_function = TripodFinger,
                                                        L = config.L, l = config.l, e1 = config.e1, 
                                                        e2 = config.e2, e3 = config.e3, n = config.n, 
                                                        d = [getattr(config, 'd'+ str(i)) for i in range(config.n)], 
                                                        w = config.w, lc = config.lc))
    rootNode.Modelling.addChild(actuatedFinger)

    
    # Object in the scene
    if config.use_object:
        rootNode.Modelling.addChild('Obstacle')

        
        if config.object_shape == "hexagon_horizontal":
            
            init_translation = np.array([-1e-3, 0, 0])# For aligning the object on x-y axis.
            sphObst = cylinder_lib.Cylinder(parent=rootNode.Modelling.Obstacle, 
                            translation= init_translation + np.array([config.distanceObject, 0.0, 65.0e-3]), 
                            surfaceMeshFileName='Models/TripodFinger/Meshes/ServoMeshes/hexagon_horizontal.stl',
                            MOscale=10e-3,
                            uniformScale=0.001,
                            totalMass=0.032,
                            isAStaticObject=True)
            
            sphObst.mass.showAxisSizeFactor = 1e-2
            sphObst.mstate.name = 'dofs'


            # Fix the object in space
            fixing_box_lib.FixingBox(rootNode.Modelling.Obstacle, sphObst, 
                                     translation = init_translation + np.array([config.distanceObject+5.0e-3, 0.0, 100.0e-3]),
                                scale=[10e-3, 10e-3, 10e-3])
            rootNode.Modelling.Obstacle.FixingBox.BoxROI.drawBoxes = True

            # Wanted contact location for the object
            ContactLocation = rootNode.Modelling.Obstacle.addChild('ContactLocation')
            ContactLocation.addObject('MechanicalObject',
                            name='dofs',
                            size=1,
                            template='Vec3d',
                            showObject=True,
                            showObjectScale=2e-3,
                            drawMode = 1,
                            showColor = "green",
                            translation2=[config.distanceObject, 0, 65.0e-3])


        elif config.object_shape == "sphere":
            init_translation = np.array([10e-3, 0, 0])# For aligning the object on x-y axis.
            sphObst = cylinder_lib.Cylinder(parent=rootNode.Modelling.Obstacle, translation= init_translation + np.array([config.distanceObject, 0.0, 80.0e-3]),
                            surfaceMeshFileName='Models/TripodFinger/Meshes/ServoMeshes/sphere.stl',
                            MOscale=10e-3,
                            uniformScale=2.0,
                            totalMass=0.032,
                            isAStaticObject=True)
            
            sphObst.mass.showAxisSizeFactor = 1e-2
            sphObst.mstate.name = 'dofs'


            # Fix the object in space
            fixing_box_lib.FixingBox(rootNode.Modelling.Obstacle, sphObst, 
                                     translation = init_translation + np.array([config.distanceObject+5.0e-3, 0.0, 80.0e-3]),
                                scale=[10e-3, 10e-3, 10e-3])
            rootNode.Modelling.Obstacle.FixingBox.BoxROI.drawBoxes = True

            # Wanted contact location for the object
            ContactLocation = rootNode.Modelling.Obstacle.addChild('ContactLocation')
            ContactLocation.addObject('MechanicalObject',
                            name='dofs',
                            size=1,
                            template='Vec3d',
                            showObject=True,
                            showObjectScale=2e-3,
                            drawMode = 1,
                            showColor = "green",
                            translation2=[config.distanceObject, 0, 80.0e-3])



        else:  #cylinder by default
            # The most positive x side of the finger is located at x = 3mm, and the radius of the obstacle is 5mm
            # So to be at 30mm from the finger, the obstacle center should be at x = 3 + 30 + 5 mm
            # distanceObject represents the 3 + 30 mm
            # cylObst = cylinder_lib.Cylinder(parent=rootNode.Modelling.Obstacle, translation=[config.distanceObject+5.0e-3, 0.0, 30.0e-3],
            #                 surfaceMeshFileName='Models/TripodFinger/Meshes/ServoMeshes/cylinder.stl',
            #                 MOscale=10e-3,
            #                 uniformScale=1.0,
            #                 totalMass=0.032,
            #                 isAStaticObject=True)
            
            # The most positive x side of the finger is located at x = 3mm, and the radius of the obstacle is 34mm
            # So to be at 30mm from the finger, the obstacle center should be at x = 3 + 30 mm
            # distanceObject represents the 3 + 30 mm
            cylObst = cylinder_lib.Cylinder(parent=rootNode.Modelling.Obstacle, 
                            translation= np.array([config.distanceObject, 0.0, 45.0e-3]),
                            surfaceMeshFileName='Models/TripodFinger/Meshes/ServoMeshes/hexagon.stl',
                            MOscale=10e-3,
                            uniformScale=0.001, # Going from mm to m
                            totalMass=0.032,
                            isAStaticObject=True)
            
            cylObst.mass.showAxisSizeFactor = 1e-2
            cylObst.mstate.name = 'dofs'


            # Fix the object in space
            fixing_box_lib.FixingBox(rootNode.Modelling.Obstacle, cylObst, translation=[config.distanceObject+5.0e-3, 0.0, 100.0e-3],
                                scale=[10e-3, 10e-3, 10e-3])
            rootNode.Modelling.Obstacle.FixingBox.BoxROI.drawBoxes = True

            # Wanted contact location for the object
            ContactLocation = rootNode.Modelling.Obstacle.addChild('ContactLocation')
            ContactLocation.addObject('MechanicalObject',
                            name='dofs',
                            size=1,
                            template='Vec3d',
                            showObject=True,
                            showObjectScale=2e-3,
                            drawMode = 1,
                            showColor = "green",
                            translation2=[config.distanceObject+5.0e-3-0.5*10e-3, 0, 100.0e-3])


    # Add the simulated elements to the Simulation node
    rootNode.Simulation.addChild(actuatedFinger.RigidifiedStructure.DeformableParts)
    rootNode.Simulation.addChild(actuatedFinger.ActuatedArm)
    
    # Temporary addition to have the system correctly built in SOFA
    # Will no longer be required in SOFA v22.12
    rootNode.Simulation.addObject('MechanicalMatrixMapper',
                            name="deformableAndFreeCenterCoupling",
                            template='Vec1,Vec3',
                            object1=actuatedFinger.ActuatedArm.ServoMotor.Articulation.dofs.getLinkPath(),
                            object2=actuatedFinger.RigidifiedStructure.DeformableParts.dofs.getLinkPath(),
                            nodeToParse=actuatedFinger.RigidifiedStructure.DeformableParts.ElasticMaterialObject.getLinkPath())

    ##################
    ### Controller ###                            
    ##################
    rootNode.addObject(FitnessEvaluationController(name="FitnessEvaluationController", rootNode=rootNode, config=config, actuator=rootNode.Modelling.ActuatedFinger.ActuatedArm))
    
    return rootNode



