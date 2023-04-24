__authors__ = "tnavez, qpeyron"
__contact__ = "tanguy.navez@inria.fr, quentin.peyron@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Feb 09 2023"

import os
import Sofa
from stlib3.scene import Scene

class ServoMotor(Sofa.Prefab):
    """A S90 servo motor

    This prefab is implementing a S90 servo motor.
    https://servodatabase.com/servo/towerpro/sg90

    The prefab ServoMotor is composed of:
    - a visual modeli
    - a mechanical model composed two rigds. One rigid is for the motor body
      while the other is implementing the servo rotating wheel.

    The prefab has the following parameters:
    - translation           to change default location of the servo (default [0.0,0.0,0.0])
    - rotation              to change default rotation of the servo (default [0.0,0.0,0.0,1])
    - scale                 to change default scale of the servo (default 1)
    - showServo             to control wether a visual model of the motor is added (default True)
    - showWheel             to control wether the rotation axis of the motor is displayed (default False)

    The prefab has the following properties:
    - angle         use this to specify the angle of rotation of the servo motor
    - angleLimits   use this to set a min and max value for the servo angle rotation
    - position      use this to specify the position of the servo motor

    Example of use in a Sofa scene:

    def addScene(root):
        ...
        servo = ServoMotor(root)

        ## Direct access to the components
        servo.angle.value = 1.0
    """
    prefabParameters = [
        {'name': 'rotation', 'type': 'Vec3d', 'help': 'Rotation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'translation', 'type': 'Vec3d', 'help': 'Translation', 'default': [0.0, 0.0, 0.0]},
        {'name': 'scale3d', 'type': 'Vec3d', 'help': 'Scale 3d', 'default': [1.0e-3, 1.0e-3, 1.0e-3]}]

    prefabData = [
        {'name': 'minAngle', 'help': 'min angle of rotation (in radians)', 'type': 'float', 'default': -100},
        {'name': 'maxAngle', 'help': 'max angle of rotation (in radians)', 'type': 'float', 'default': 100},
        {'name': 'angleIn', 'help': 'angle of rotation (in radians)', 'type': 'float', 'default': 0},
        {'name': 'angleOut', 'help': 'angle of rotation (in degree)', 'type': 'float', 'default': 0}
    ]

    def __init__(self, *args, **kwargs):
        Sofa.Prefab.__init__(self, *args, **kwargs)

    def init(self):
        # Servo body
        servoBody = self.addChild('ServoBody')
        servoBody.addObject('MechanicalObject', name='dofs', template='Rigid3', position=[[0., 0., 0., 0., 0., 0., 1.]],
                            translation=list(self.translation.value), rotation=list(self.rotation.value),
                            scale3d=list(self.scale3d.value))
        servoBody.addObject('FixedConstraint', indices=0)
        servoBody.addObject('UniformMass', totalMass=0.01)

        visual = servoBody.addChild('VisualModel')
        visual.addObject('MeshSTLLoader', name='loader', filename='Models/TripodFinger/Meshes/ServoMeshes/SG90_servomotor_finger.stl', scale=1e-3,
                         rotation=[0.0, -90.0, 0.0], translation=[-12.0e-3, -5.0e-3, 0.0])
        visual.addObject('MeshTopology', src='@loader')
        visual.addObject('OglModel', color=[0.15, 0.45, 0.75, 0.7], writeZTransparent=True)
        visual.addObject('RigidMapping', index=0)

        # Servo wheel
        angle = self.addChild('Articulation')
        angle.addObject('MechanicalObject', name='dofs', template='Vec1', position=[[self.getData('angleIn')]],
                        rest_position=self.getData('angleIn').getLinkPath())
        angle.addObject('RestShapeSpringsForceField', points=0, stiffness=1e9)
        angle.addObject('UniformMass', totalMass=0.01)

        servoWheel = angle.addChild('ServoWheel')
        servoWheel.addObject('MechanicalObject', name='dofs', template='Rigid3d',
                             position=[[0., 0., 0., 0., 0., 0., 1.], [0., 0., 0., 0., 0., 0., 1.]],
                             translation=list(self.translation.value), rotation=list(self.rotation.value),
                             scale3d=list(self.scale3d.value))
        servoWheel.addObject('ArticulatedSystemMapping', input1="@../dofs", input2="@../../ServoBody/dofs",
                             output="@./")

        articulationCenter = angle.addChild('ArticulationCenter')
        articulationCenter.addObject('ArticulationCenter', parentIndex=0, childIndex=1, posOnParent=[0., 0., 0.],
                                     posOnChild=[0., 0., 0.])
        articulation = articulationCenter.addChild('Articulations')
        articulation.addObject('Articulation', translation=False, rotation=True, rotationAxis=[1, 0, 0],
                               articulationIndex=0)
        angle.addObject('ArticulatedHierarchyContainer', printLog=False)

        # The output
        self.angleOut.setParent(angle.dofs.position)
