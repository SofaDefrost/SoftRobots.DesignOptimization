__authors__ = "tnavez, qpeyron"
__contact__ = "tanguy.navez@inria.fr, quentin.peyron@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Feb 09 2023"

import Sofa

from stlib3.physics.mixedmaterial import Rigidify
from stlib3.physics.collision import CollisionMesh
from stlib3.components import addOrientedBoxRoi
from Components.fixing_box import FixingBox
from Components.actuated_arm import ActuatedArm
from Components.elastic_material_object import ElasticMaterialObject

from splib3.constants import Key
import math

class ActuatedFinger(Sofa.Prefab):

    prefabParameters = [
        {"name": "rotation", "type": "Vec3d", "help": "Rotation in base frame", "default": [0.0, 0.0, 0.0]},
        {"name": "translation", "type": "Vec3d", "help": "Translation in base frame",
         "default": [0.0, 0.0, 0.0]},
         {"name": "youngModulus", "type": "double", "help": "Young modulus for simulating the finger",
         "default": 7e6},
         {"name": "poissonRatio", "type": "double", "help": "Poisson ratio for simulating the finger",
         "default": 0.45},
         {"name": "volumeMeshFileName", "type": "string", "help": "Path to the mesh used for computing volumetric mesh", 
        "default": "Models/TripodFinger/Meshes/finger.msh"},
        {"name": "stlMeshFileName", "type": "string", "help": "Path to the surface mesh", 
        "default": "Models/TripodFinger/Meshes/finger.stl"},
        {"name": "stlMeshFileNameIn1", "type": "string", "help": "Path to the surface mesh", 
        "default": "Models/TripodFinger/Meshes/finger_surface_contact_in1.stl"},
        {"name": "stlMeshFileNameIn2", "type": "string", "help": "Path to the surface mesh", 
        "default": "Models/TripodFinger/Meshes/finger_surface_contact_in2.stl"},
        {"name": "stlMeshFileNameOut", "type": "string", "help": "Path to the surface mesh", 
        "default": "Models/TripodFinger/Meshes/finger_surface_contact_out.stl"}
    ]


    
    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)


    # Construct the actuated finger
    def init(self):

        # Load the finger mesh and create an elastic body from it
        self.elasticMaterial = self.elasticBody()
        self.ElasticBody.init()

        # Load a servo motor
        arm = self.addChild(ActuatedArm(name="ActuatedArm", rotation=[90.0, 0, 90.0], translation=[0, 0, 0]))
        arm.ServoMotor.Articulation.dofs.position.value = [[arm.angleIn.value]]  # Initialize the angle
        arm.ServoMotor.minAngle.value = -2.02
        arm.ServoMotor.maxAngle.value = -0.025

        # Define a region of interest to rigidify the nodes of the finger mesh clamped in the servo arm
        box = addOrientedBoxRoi(self,
                                name="boxROIclamped",
                                position=[list(i) for i in self.elasticMaterial.dofs.rest_position.value],
                                translation=[-25e-3, 0.0, 0.0],
                                eulerRotation=[0.0, 0.0, 90.0],
                                scale=[45e-3, 15e-3, 30e-3])
        box.drawBoxes = True
        box.init()
        
        # Get the indices of the finger mesh in the ROI, and the ROI frame
        indices = [[ind for ind in box.indices.value]]
        frame = [[0, 0, 0, 0, 0, 0, 1]]

        # Rigidify the finger nodes in the ROI. Create a Rigidified object and set up a spring force
        # field to constrain the nodes to stay in the rest shape
        rigidifiedStruct = Rigidify(self, self.elasticMaterial, groupIndices=indices, frames=frame,
                                    name="RigidifiedStructure")

        servoArm = arm.ServoMotor.Articulation.ServoWheel.ServoArm
        servoArm.addChild(rigidifiedStruct.RigidParts)
        servoArm.RigidParts.addObject('RigidRigidMapping', index=0, input=servoArm.dofs.getLinkPath())

        # Add a fixing box to constrain the other part of the finger        
        FixingBox(self, self.elasticMaterial, translation=[5.0e-3, 0.0, 14.0e-3], scale=[15e-3, 25e-3, 6e-3])
        self.FixingBox.BoxROI.drawBoxes = True

        # Add collision models, to compute collision between the finger and the object,
        # and the inner surfaces of the left and right walls
        self.addCollision()

    def elasticBody(self):
        # Create a body as a child of the parent (the actuated finger)
        body = self.addChild("ElasticBody")

        # Create an ElasticMaterialObject, which import a mesh, assign it dofs  and mechanical properties
        # All the properties are expressed in SI units. The dimensions in the generated mesh are in meter,
        # the young modulus in Pascal ...
        # Data from https://www.frontiersin.org/articles/10.3389/frobt.2021.615991/full
        # The rotation and translation are adjusted so that the mesh is correctly positioned wrt the servo motor
        e = body.addChild(ElasticMaterialObject(
            volumeMeshFileName=self.volumeMeshFileName.value,
            topoMesh="tetrahedron",
            scale=[1, 1, 1],
            totalMass=0.015,
            youngModulus=self.youngModulus.value,
            poissonRatio=self.poissonRatio.value,
            rotation=[90.0, 0.0, 0.0],
            translation=[-30.0e-3, 9.0e-3, 18.0e-3]))
        
        # Add now a visual model to the flexible part
        visual = body.addChild("VisualFinger")
        # Load the STL file for the visualization, the rotation and translation must
        # fit the one for the ElasticMaterialObject
        visual.addObject("MeshSTLLoader", name="visualLoader", filename=self.stlMeshFileName.value, rotation=[90.0, 0.0, 0.0],
                         translation=[-30.0e-3, 9.0e-3, 18.0e-3])
        visual.addObject("OglModel", name="renderer", src="@visualLoader", color=[1.0, 1.0, 1.0, 0.5])

        # Link the dofs of the 3D mesh and the visual model
        visual.addObject("BarycentricMapping", input=e.dofs.getLinkPath(), output=visual.renderer.getLinkPath())

        return e

    def addCollision(self):
        # Add a collision model
        CollisionMesh(self.elasticMaterial, name='SelfCollisionMesh1',
                                   surfaceMeshFileName=self.stlMeshFileNameIn1.value,
                                   rotation=[90.0, 0.0, 0.0], translation=[-30.0e-3, 9.0e-3, 18.0e-3])

        CollisionMesh(self.elasticMaterial, name='SelfCollisionMesh2',
                                   surfaceMeshFileName=self.stlMeshFileNameIn2.value,
                                   rotation=[90.0, 0.0, 0.0], translation=[-30.0e-3, 9.0e-3, 18.0e-3])

        CollisionMesh(self.elasticMaterial, name='CollisionMeshOut',
                                   surfaceMeshFileName=self.stlMeshFileNameOut.value,
                                   rotation=[90.0, 0.0, 0.0], translation=[-30.0e-3, 9.0e-3, 18.0e-3])


class FingerController(Sofa.Core.Controller):

    def __init__(self, *args, **kwargs):
        # These are needed (and the normal way to override from a python class)
        Sofa.Core.Controller.__init__(self, *args, **kwargs)

        self.node = kwargs["node"]
        self.duration = 3.0
        self.time = 0.0
        self.objectDof = kwargs["objectDof"]
        self.actuator = kwargs["actuator"]
        self.forceContact = 0.0
        self.numContact = 0
        print(self.node.getRoot())
        # Computation of the contact force applied on the object to grasp
        self.node.getRoot().GenericConstraintSolver.computeConstraintForces.value = True

    def onKeypressedEvent(self, event):
        key = event['key']
        if key == Key.P:
            print("Number of contact points: " + str(self.numContact))
            print("Norm of the contact force: " + str(self.forceContact))
    def evaluateForce(self):
        return self.forceContact

    def onAnimateBeginEvent(self, eventType):

        # Update of the servomotor angular displacement
        # Rotation of pi/6 over self.duration (5s initially)
        angularStep = math.pi / 6
        angleInit = 0
        self.time += self.node.dt.value
        if self.time < self.duration:
            self.actuator.ServoMotor.angleIn = angleInit + angularStep * self.time / self.duration
        else:
            self.actuator.ServoMotor.angleIn = angleInit + angularStep

        # Computation of the contact force applied on the object to grasp
        contactForces = self.node.getRoot().GenericConstraintSolver.constraintForces.value

        # print the number of nodes in contact and the norm of the largest contact force
        self.numContact = 0
        self.forceContact = 0
        for contact in contactForces[0:-1:3]:
            if contact > 0:
                self.numContact += 1
                self.forceContact += contact
        self.forceContact /= self.node.dt.value
