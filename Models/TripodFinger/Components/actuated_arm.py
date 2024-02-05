__authors__ = "tnavez, qpeyron"
__contact__ = "tanguy.navez@inria.fr, quentin.peyron@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Feb 09 2023"

import Sofa
from stlib3.visuals import VisualModel
from Components.s90_servo import ServoMotor

class ServoArm(Sofa.Prefab):
    """ServoArm is a reusable sofa model of a servo arm for the S90 servo motor

       Parameters:
            parent:        node where the ServoArm will be attached
            mappingInput:  the rigid mechanical object that will control the orientation of the servo arm
            indexInput: (int) index of the rigid the ServoArm should be mapped to
    """

    prefabParameters = [
        {'name': 'mappingInputLink', 'type': 'string',
         'help': 'the rigid mechanical object that will control the orientation of the servo arm', 'default': ''},
        {'name': 'indexInput', 'type': 'int', 'help': 'index of the rigid the ServoArm should be mapped to',
         'default': 1}]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):
        self.addObject('MechanicalObject',
                       name='dofs',
                       size=1,
                       template='Rigid3',
                       showObject=False,
                       showObjectScale=5e-3,
                       translation2=[0, 0, -25e-3])

    def setRigidMapping(self, path):
        self.addObject('RigidRigidMapping', name='mapping', initialPoints=path, index=self.indexInput.value) 
	
        visual_mesh_path = 'Models/TripodFinger/Meshes/ServoMeshes/SG90_servoarm.stl'
        visual = self.addChild(VisualModel(visualMeshPath = visual_mesh_path, translation=[0, 0, 25.0e-3],
                                           rotation=[-90, 0, 0],
                                           scale=[1.0e-3, 1.0e-3, 1.0e-3], color=[1., 1., 1., 0.75]))
        visual.OglModel.writeZTransparent = True
        visual.addObject('RigidMapping', name='mapping')


class ActuatedArm(Sofa.Prefab):
    """ActuatedArm is a reusable sofa model of a S90 servo motor and the tripod actuation arm.
           Parameters:
             - translation the position in space of the structure
             - eulerRotation the orientation of the structure

           Structure:
           Node : {
                name : 'ActuatedArm'
                MechanicalObject     // Rigid position of the motor
                ServoMotor           // The s90 servo motor with its actuated wheel
                ServoArm             // The actuation arm connected to ServoMotor.ServoWheel
            }
    """
    prefabParameters = [
        {'name': 'rotation', 'type': 'Vec3d', 'help': 'Rotation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'translation', 'type': 'Vec3d', 'help': 'Translation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'scale', 'type': 'Vec3d', 'help': 'Scale 3d', 'default': [1.0e-3, 1.0e-3, 1.0e-3]}]

    prefabData = [
        {'name': 'angleIn', 'group': 'ArmProperties', 'help': 'angle of rotation (in radians) of the arm',
         'type': 'float', 'default': 0},
        {'name': 'angleOut', 'group': 'ArmProperties', 'type': 'float', 'help': 'angle of rotation (in radians) of '
                                                                                'the arm', 'default': 0}
    ]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):
        self.servomotor = self.addChild(ServoMotor(name="ServoMotor", translation=self.translation.value,
                                                   rotation=self.rotation.value))
        self.servoarm = self.servomotor.Articulation.ServoWheel.addChild(ServoArm(name="ServoArm"))
        self.servoarm.setRigidMapping(self.ServoMotor.Articulation.ServoWheel.dofs.getLinkPath())

        # add a public attribute and connect it to the private one.
        self.ServoMotor.angleIn.setParent(self.angleIn)

        # add a public attribute and connect it to the internal one.
        self.angleOut.setParent(self.ServoMotor.angleOut)
