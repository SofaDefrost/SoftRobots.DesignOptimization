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
        
        # Computation of the contact force applied on the object to grasp
        self.rootNode.getRoot().GenericConstraintSolver.computeConstraintForces.value = True
        self.angularStep = math.pi / 6
        self.angleInit = 0
        
        # Objective evaluation variables
        self.current_iter = 0
        current_objectives = self.config.get_currently_assessed_objectives()
        
        self.max_iter = max([self.config.get_objective_data()[current_objectives[i]][1] for i in range(len(current_objectives))])
        self.iter_to_angular_disp = self.max_iter - 10
        
    def onAnimateBeginEvent(self, dt):
        self.current_iter += 1

        # Incremental update of servomotor angular position
        if self.current_iter < self.iter_to_angular_disp:
            self.actuator.ServoMotor.angleIn = self.angleInit + self.angularStep * self.current_iter / self.iter_to_angular_disp
        else:
            self.actuator.ServoMotor.angleIn = self.angleInit + self.angularStep
        
        ### Evaluated forces applied on the cylinder
        if self.current_iter == self.max_iter:

            current_objectives_name = self.config.get_currently_assessed_objectives()   

            self.MotorTorque=self.actuator.ServoMotor.ServoBody.dofs.force.value[0][4]
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
            
            self.forceContactN = (forceX**2+forceY**2+forceZ**2)**0.5
            self.forceContactN /= self.rootNode.dt.value
                                    
            print("Torque="+str(self.MotorTorque))
                
            for i in range(len(current_objectives_name)):

                current_objective_name =  current_objectives_name[i]

                # Contact Force along X-axis
                if "ContactForceX" == current_objective_name:
                    self.forceContactX = self.forceContactX / self.rootNode.dt.value
                    print("The Contact Force along X:"+str(self.forceContactX))
                    self.objectives.append(self.forceContactX)

                # Norm of the Contact Force
                if "ContactForceNorm" == current_objective_name:
                    self.forceContactN = (forceX**2+forceY**2+forceZ**2)**0.5 / self.rootNode.dt.value
                    print("Norm of the Contact Force:"+str(self.forceContactN))
                    self.objectives.append(self.forceContactN)

                # Mass of material needed for building the Tripod Finger
                if "Mass" == current_objective_name:
                    volumeMeshFileName = self.config.get_mesh_filename(mode = "Surface", refine = False, 
                                                        generating_function = TripodFinger,
                                                        L = self.config.L, l = self.config.l, e1 = self.config.e1, 
                                                        e2 = self.config.e2, e3 = self.config.e3, n = self.config.n, 
                                                        d = [getattr(self.config, 'd'+ str(i)) for i in range(self.config.n)], 
                                                        w = self.config.w, lc = self.config.lc)
                    mesh = pyvista.read(volumeMeshFileName)
                    self.mass = self.config.rho*mesh.volume
                    print("Mass of the Finger:", self.mass)
                    self.objectives.append(self.mass)


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
    rootNode.addObject('LocalMinDistance',alarmDistance=15e-3, contactDistance=0.1e-3, angleCone=0.01)
    
    # Set up the pipeline for the collision and mechanics computation
    rootNode.addObject('GenericConstraintSolver', tolerance="1e-6", maxIterations="10000")
    rootNode.Simulation.addObject('GenericConstraintCorrection')
    rootNode.Simulation.addObject('EulerImplicitSolver', name='TimeIntegrationSchema')
    rootNode.Simulation.addObject('SparseLDLSolver', name='LinearSolver', template="CompressedRowSparseMatrixMat3x3d")

    # Create one actuated finger
    actuatedFinger = actuated_finger_lib.ActuatedFinger(
        stlMeshFileNameIn1 = config.get_mesh_filename(mode = "Surface", refine = False, 
                                                        generating_function = ContactSurfaceIn1,
                                                        L = config.L, e1 = config.e1, e2 = config.e2, 
                                                        w = config.w, lc = config.lc), 
        stlMeshFileNameIn2 = config.get_mesh_filename(mode = "Surface", refine = False, 
                                                        generating_function = ContactSurfaceIn2,
                                                        L = config.L, l = config.l, e1 = config.e1, 
                                                        e2 = config.e2, e3 = config.e3, n = config.n, 
                                                        d = [getattr(config, 'd'+ str(i)) for i in range(config.n)], 
                                                        w = config.w, lc = config.lc), 
        stlMeshFileNameOut = config.get_mesh_filename(mode = "Surface", refine = False, 
                                                        generating_function = ContactSurfaceOut,
                                                        L = config.L, l = config.l, e1 = config.e1, 
                                                        e2 = config.e2, e3 = config.e3, n = config.n, 
                                                        d = [getattr(config, 'd'+ str(i)) for i in range(config.n)], 
                                                        w = config.w, lc = config.lc),
        stlMeshFileName = config.get_mesh_filename(mode = "Surface", refine = False, 
                                                        generating_function = TripodFinger,
                                                        L = config.L, l = config.l, e1 = config.e1, 
                                                        e2 = config.e2, e3 = config.e3, n = config.n, 
                                                        d = [getattr(config, 'd'+ str(i)) for i in range(config.n)], 
                                                        w = config.w, lc = config.lc), 
        volumeMeshFileName = config.get_mesh_filename(mode = "Volume", refine = False, 
                                                        generating_function = TripodFinger,
                                                        L = config.L, l = config.l, e1 = config.e1, 
                                                        e2 = config.e2, e3 = config.e3, n = config.n, 
                                                        d = [getattr(config, 'd'+ str(i)) for i in range(config.n)], 
                                                        w = config.w, lc = config.lc))
    rootNode.Modelling.addChild(actuatedFinger)

    
    # Object in the scene
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



