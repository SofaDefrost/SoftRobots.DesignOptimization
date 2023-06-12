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
        self.angularStep = math.pi / 6 
        self.angleInit = 0
        
        # Objective evaluation variables
        self.current_iter = 0
        current_objectives = self.config.get_currently_assessed_objectives()
        
        self.max_iter = max([self.config.get_objective_data()[current_objectives[i]][1] for i in range(len(current_objectives))])
        self.iter_to_angular_disp = self.max_iter - 10

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
        self.current_iter += 1

        # Incremental update of servomotor angular position
        if self.current_iter < self.iter_to_angular_disp:
            self.actuator.ServoMotor.angleIn = self.angleInit + self.angularStep * self.current_iter / self.iter_to_angular_disp
        else:
            self.actuator.ServoMotor.angleIn = self.angleInit + self.angularStep

        ### Initial coordinates of collision meshes vertices
        if self.current_iter == 1:
            self.coords_finger_collis_init = copy.deepcopy(self.contact_finger_collis.position.value)
            if self.config.use_object:
                self.coords_object_collis_init = copy.deepcopy(self.contact_cylinder_collis.position.value)

        ### Evaluated forces applied on the cylinder
        if self.current_iter == self.max_iter:

            current_objectives_name = self.config.get_currently_assessed_objectives()   

            # self.MotorTorqueOld = self.actuator.ServoMotor.ServoBody.dofs.force.value[0][4]
            # print("Old Torque="+str(self.MotorTorqueOld))

            # As the torque is modeled as a Spring, we compute the torque as k * (theta - theta_0)
            k = self.actuator.ServoMotor.Articulation.RestShapeSpringsForceField.stiffness.value
            theta_0 = self.actuator.ServoMotor.Articulation.dofs.rest_position.value
            theta = self.actuator.ServoMotor.Articulation.dofs.position.value
            self.MotorTorque = k * (theta - theta_0)
            print("Torque="+str(self.MotorTorque))

            if self.config.use_object:
                contactForces = self.rootNode.getRoot().GenericConstraintSolver.constraintForces.value
                constraint= self.rootNode.Modelling.Obstacle.Cylinder.collision.MechanicalObject.constraint.value.split('\n')[0:-1]
                indices_constraint=[]
                self.numContact=len(constraint)
                self.forceContactX=0
                forceX,forceY,forceZ=0,0,0
                
                for i in range(len(constraint)):
                    indices_constraint.append([int(constraint[i].split(' ')[0]),float(constraint[i].split(' ')[3]),float(constraint[i].split(' ')[4]),float(constraint[i].split(' ')[5])])
                    norm=(indices_constraint[i][1]**2+indices_constraint[i][2]**2+indices_constraint[i][3]**2)**0.5
                    ###### Evaluate the normal force
                    self.forceContactX += 1/norm*contactForces[indices_constraint[i][0]]*indices_constraint[i][1] 
                    ####### Evaluate force norm
                    forceX+=1/norm*contactForces[indices_constraint[i][0]]*indices_constraint[i][1]
                    forceY+=1/norm*contactForces[indices_constraint[i][0]]*indices_constraint[i][2]
                    forceZ+=1/norm*contactForces[indices_constraint[i][0]]*indices_constraint[i][3]
            
            if self.config.use_object:
                # In SOFA, force*dt is stored in lambda vectors for ease of use. 
                # We remove the dt part for obtaining only the force
                self.forceContactX /= self.rootNode.dt.value
                forceX /= self.rootNode.dt.value 
                forceY /= self.rootNode.dt.value
                forceZ /= self.rootNode.dt.value

                # Build dict of constraint data from solver
                constraints_data = {}
                for constraint in indices_constraint:
                    constraints_data[constraint[0]] = constraint[1:]
           

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
                    print("The Contact Force along X: "+str(self.forceContactX))
                    self.objectives.append(self.forceContactX)

                # Norm of the Contact Force
                if "ContactForceNorm" == current_objective_name:
                    self.forceContactN = (forceX**2+forceY**2+forceZ**2)**0.5 
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
                        norm = (constraints_data[id][0]**2+constraints_data[id][1]**2+constraints_data[id][2]**2)**0.5 
                        curr_forceX = abs(1/norm*contactForces[id]*constraints_data[id][0]) / self.rootNode.dt.value
                        self.contactForceXLocationPenalty += (abs(TARGET[2] - collision_coordinates[id][2]) - THRESHOLD) * curr_forceX
                        sum_forceX += curr_forceX

                    # Normalize by sum of forces
                    # self.contactForceXLocationPenalty = self.contactForceXLocationPenalty / sum_forceX

                    print("Contact Location Penalty along x-axis: "
                        + str(self.contactForceXLocationPenalty))
                    self.objectives.append(self.contactForceXLocationPenalty)

                # print("idx_finger_collis:", idx_constraint_finger_collis)
                # print("idx_object_collis:", idx_constraint_object_collis)
                # print("links_finger_collis:", links_finger_collis)
                # print("links_object_collis:", links_object_collis)
                # print("coord_finger_collis:", coord_finger_collis)
                # print("coord_object_collis:", coord_object_collis)


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
    frictionCoef=0.1
    rootNode.addObject('RuleBasedContactManager', responseParams="mu="+str(frictionCoef),
                                                    name='Response', response='FrictionContactConstraint')#frictionCoef=0.1
    rootNode.addObject('LocalMinDistance',alarmDistance=25e-3, contactDistance=0.1e-3, angleCone=0.01)
    
    # Set up the pipeline for the collision and mechanics computation
    rootNode.addObject('GenericConstraintSolver', tolerance="1e-6", maxIterations="10000")
    rootNode.Simulation.addObject('GenericConstraintCorrection')
    rootNode.Simulation.addObject('EulerImplicitSolver', name='TimeIntegrationSchema')
    rootNode.Simulation.addObject('SparseLDLSolver', name='LinearSolver', template="CompressedRowSparseMatrixMat3x3d")

    # Create one actuated finger
    actuatedFinger = actuated_finger_lib.ActuatedFinger(
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

        cylObst = cylinder_lib.Cylinder(parent=rootNode.Modelling.Obstacle, translation=[30.0e-3, 0.0, 50.0e-3],
                            surfaceMeshFileName='Models/TripodFinger/Meshes/ServoMeshes/cylinder.stl',
                            MOscale=10e-3,
                            uniformScale=0.5,
                            totalMass=0.032,
                            isAStaticObject=True)
        cylObst.mass.showAxisSizeFactor = 1e-2
        cylObst.mstate.name = 'dofs'

        # Fix the object in space
        fixing_box_lib.FixingBox(rootNode.Modelling.Obstacle, cylObst, translation=[30.0e-3, 0.0, 70.0e-3],
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
                        translation2=[30.0e-3-0.5*10e-3, 0, 70.0e-3])

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



