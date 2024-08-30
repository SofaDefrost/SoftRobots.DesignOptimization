# -*- coding: utf-8 -*-
"""Config for the SensorFinger"""

__authors__ = "sescaidanavarro, tnavez"
__contact__ = "stefan.escaida@uoh.cl, tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Oct 28 2022"

import math
import numpy as np

from BaseFitnessEvaluationController import BaseFitnessEvaluationController

from Generation import Cavity, Finger

from MoldGeneration import MoldBox, MoldLid, MoldForCork, FingerClamp

class FitnessEvaluationController(BaseFitnessEvaluationController):   
    
    def __init__(self, *args, **kwargs):

        print('>>> Start Init SOFA scene ...')

        super(FitnessEvaluationController,self).__init__(*args, **kwargs)

        self.ModelNode = self.rootNode.model        
        self.CableConstraint = self.ModelNode.cables.cable1.CableConstraint
        self.ReferenceMO = self.rootNode.ReferenceMONode.ReferenceMO
        self.StartPosition = np.array(self.ReferenceMO.position.value[0])
        self.StartAngle = math.acos( np.abs(self.StartPosition[2]) / np.linalg.norm(self.StartPosition)) 
        self.FollowingMO = self.rootNode.model.FollowingMONode.FollowingMO
        
        # Cavities
        self.SurfacePressureConstraint1 = self.ModelNode.Cavity01.SurfacePressureConstraint        
        self.SurfacePressureConstraint2 = self.ModelNode.Cavity02.SurfacePressureConstraint
        
        # Objective evaluation variables
        self.current_iter = 0
        current_objectives = self.config.get_currently_assessed_objectives()
        self.max_iter = max([self.config.get_objective_data()[current_objectives[i]][1] for i in range(len(current_objectives))])
        
        
        print('>>> ... End')
        

    def onAnimateBeginEvent(self, dt):
        
        self.current_iter += 1
        
        if self.current_iter == self.max_iter:            
            
            current_objectives_names = self.config.get_currently_assessed_objectives()

            for i in range(len(current_objectives_names)):

                current_objective_name =  current_objectives_names[i]

                # Sensibility metrics. Reflects the efficiency of a pressure sensor.
                if "PressureSensibility" == current_objective_name:
                    CavityVolume = self.ModelNode.Cavity01.SurfacePressureConstraint.cavityVolume.value
                    Cavity01VolumeGrowth = self.SurfacePressureConstraint1.volumeGrowth.value
                    Growth = np.abs(Cavity01VolumeGrowth/CavityVolume)
                    print("Growth differential: ", Growth)
                    self.objectives.append(Growth)
                    
                # Sensibility metrics to volume variation. Reflects the efficiency of a volume sensor.
                if "VolumeSensibility" == current_objective_name:
                    Cavity01VolumeGrowth = self.SurfacePressureConstraint1.volumeGrowth.value
                    Growth = np.abs(Cavity01VolumeGrowth)
                    print("Growth differential: ", Growth)
                    self.objectives.append(Growth)
                    
                # Sensibility metrics to volume variation. Reflects the efficiency of a volume sensor but taking into account the initial volume 
                # This tweaks is to avoid obtaining non feasible design with too small cavities 
                if "PenalizedVolumeSensibility" == current_objective_name:
                    InitialCavityVolume = self.ModelNode.Cavity01.SurfacePressureConstraint.initialCavityVolume.value
                    Cavity01VolumeGrowth = self.SurfacePressureConstraint1.volumeGrowth.value
                    Growth = np.abs(Cavity01VolumeGrowth - InitialCavityVolume)
                    print("Growth differential: ", Growth)
                    self.objectives.append(Growth)

                # Initial cavity volume
                if "InitialVolume" == current_objective_name:
                    InitialCavityVolume = self.ModelNode.Cavity01.SurfacePressureConstraint.initialCavityVolume.value
                    print("Initial Volume ", InitialCavityVolume)
                    self.objectives.append(InitialCavityVolume)
                
                # Absolute Bending Angle 
                if "AbsoluteBendingAngle" == current_objective_name:               
                    CurrentPosition = np.array(self.FollowingMO.position.value[0])
                    Angle = np.abs(math.acos( abs(CurrentPosition[2]) / np.linalg.norm(CurrentPosition)))
                    print("Absolute angle: ", Angle)
                    self.objectives.append(Angle)
                

def createScene(rootNode, config):
    
    ###############################
    ### Import required plugins ###
    ###############################
    rootNode.addObject("RequiredPlugin", name="SoftRobots")
    rootNode.addObject("RequiredPlugin", name="SofaSparseSolver")
    rootNode.addObject("RequiredPlugin", name="SofaPreconditioner")
    rootNode.addObject("RequiredPlugin", name="SofaPython3")
    rootNode.addObject('RequiredPlugin', name='SofaOpenglVisual')
    rootNode.addObject('RequiredPlugin', name="SofaMiscCollision")
    rootNode.addObject("RequiredPlugin", name="SofaBoundaryCondition")
    rootNode.addObject("RequiredPlugin", name="SofaConstraint")
    rootNode.addObject("RequiredPlugin", name="SofaEngine")
    rootNode.addObject('RequiredPlugin', name='SofaImplicitOdeSolver')
    rootNode.addObject('RequiredPlugin', name='SofaLoader')
    rootNode.addObject('RequiredPlugin', name="SofaSimpleFem")
    rootNode.addObject('RequiredPlugin', name="SofaDeformable")
    rootNode.addObject('RequiredPlugin', name="SofaGeneralLoader")

    ##############################
    ### Visualization settings ###
    ##############################
    rootNode.addObject('LightManager')
    rootNode.addObject('PositionalLight', name="light1", color="0.8 0.8 0.8", position="0 60 50")                
    rootNode.addObject('PositionalLight', name="light2", color="0.8 0.8 0.8", position="0 -60 -50") 
    rootNode.addObject('VisualStyle', displayFlags='hideWireframe showBehaviorModels hideCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields')

    ###########################
    ### Simulation settings ###
    ###########################
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('GenericConstraintSolver', tolerance="1e-12", maxIterations="10000")

    rootNode.findData('gravity').value = [0, 0, -9810] 
    rootNode.findData('dt').value = 0.01

    model = rootNode.addChild('model')
    model.addObject('EulerImplicitSolver', name='odesolver', firstOrder=0, rayleighMass=0.1,  rayleighStiffness=0.1)
    model.addObject('SparseLDLSolver', name='precond', template = "CompressedRowSparseMatrixd")
    model.addObject('GenericConstraintCorrection')

    ##################
    ### Load model ###
    ##################
    model.addObject('MeshVTKLoader', name='loader', 
                    filename = config.get_mesh_filename(mode = "Volume", refine = 0, 
                                                        generating_function = Finger, 
                                Length = config.Length, Height = config.Height, OuterRadius = config.OuterRadius,
                                TeethRadius = config.TeethRadius, PlateauHeight = config.PlateauHeight, 
                                JointHeight = config.JointHeight, Thickness = config.Thickness, 
                                JointSlopeAngle = config.JointSlopeAngle, FixationWidth = config.FixationWidth, 
                                BellowHeight = config.BellowHeight, NBellows = config.NBellows, 
                                WallThickness = config.WallThickness, CenterThickness = config.CenterThickness,
                                CavityCorkThickness = config.CavityCorkThickness, lc = config.lc_finger, 
                                RefineAroundCavities=config.RefineAroundCavities))
    model.addObject('TetrahedronSetTopologyContainer', name='container', src='@loader')
    model.addObject('TetrahedronSetGeometryAlgorithms')
    model.addObject('MechanicalObject', name='tetras', template='Vec3d', showIndices='false', showIndicesScale='4e-5')
    model.addObject('UniformMass', totalMass='0.1')
    model.addObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio=config.PoissonRation,  youngModulus=config.YoungsModulus)

    BoxMargin = 3
    BoxCoords = [-(config.Thickness/2+BoxMargin), -BoxMargin, BoxMargin, config.Thickness/2+BoxMargin,config.Height+2*BoxMargin, -BoxMargin]
    model.addObject('BoxROI', name='boxROI', box=BoxCoords, drawBoxes=True)
    model.addObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness=1e10)               
    
    FollowingMONode = model.addChild('FollowingMONode')                
    FollowingMONode.addObject("MechanicalObject", name="FollowingMO", template="Vec3d", position=[0.0, 0, -3.0*config.Length], showObject=True, showObjectScale=20, showColor="0 0 1") 
    FollowingMONode.addObject("BarycentricMapping")

    # Effectors                                               
    for i in range(1,3):                        
        CurrentCavity = model.addChild('Cavity0'+str(i))
        BellowGap = (config.NBellows-1)*config.BellowHeight
        if i == 1:
            Z_translation = -(config.Length+BellowGap/2)
        elif i == 2:
            Z_translation = -2*(config.Length+3/4*BellowGap)
        CurrentCavity.addObject('MeshSTLLoader', name='MeshLoader', 
                                filename=config.get_mesh_filename(mode = "Surface", refine = 0, 
                                                    generating_function = Cavity,
                                    Length = config.Length, Height = config.Height, Thickness = config.Thickness, 
                                    OuterRadius = config.OuterRadius, NBellows = config.NBellows, 
                                    BellowHeight = config.BellowHeight, TeethRadius = config.TeethRadius, 
                                    WallThickness = config.WallThickness, CenterThickness = config.CenterThickness,
                                    CavityCorkThickness = config.CavityCorkThickness, PlateauHeight = config.PlateauHeight, 
                                    Z_translation = Z_translation, RefineAroundCavities = config.RefineAroundCavities))
        CurrentCavity.addObject('Mesh', name='topology', src='@MeshLoader')
        CurrentCavity.addObject('MechanicalObject', src="@topology")
        CurrentCavity.addObject('SurfacePressureConstraint', template='Vec3d', triangles='@topology.triangles')
        CurrentCavity.addObject('BarycentricMapping', name="Mapping", mapForces="false", mapMasses="false")

    # Visualization                          
    modelVisu = model.addChild('visu')
    modelVisu.addObject('MeshSTLLoader', name="loader", 
                        filename = config.get_mesh_filename(mode = "Surface", refine = 1, 
                                                    generating_function =  Finger,
                                Length = config.Length, Height = config.Height, OuterRadius = config.OuterRadius, 
                                TeethRadius = config.TeethRadius, PlateauHeight = config.PlateauHeight, 
                                JointHeight = config.JointHeight, Thickness = config.Thickness, JointSlopeAngle = config.JointSlopeAngle, 
                                FixationWidth = config.FixationWidth, BellowHeight = config.BellowHeight, 
                                NBellows = config.NBellows, WallThickness = config.WallThickness, 
                                CenterThickness = config.CenterThickness, CavityCorkThickness = config.CavityCorkThickness, 
                                lc = config.lc_finger, RefineAroundCavities=config.RefineAroundCavities))
    modelVisu.addObject('OglModel', src="@loader", scale3d=[1, 1, 1])
    modelVisu.addObject('BarycentricMapping')

    # Cable Actuator                        
    cables = model.addChild('cables')
    cable1 = cables.addChild('cable1')
    
    NSegments = 3  
    CableHeight = config.CableHeight
    CableHeightRelative = CableHeight-config.JointHeight
    LengthDiagonal = CableHeightRelative/np.cos(config.JointSlopeAngle)
    JointStandoff = LengthDiagonal*np.sin(config.JointSlopeAngle)
    
    CablePoints = np.array([])
    for i in range(NSegments):
        SegmentOffsetBase = config.Length*i
        SegmentOffsetTip  = config.Length*(i+1)
        CablePoints = np.append(CablePoints, [[0,CableHeight,-SegmentOffsetBase - JointStandoff]])
        CablePoints = np.append(CablePoints, [[0,CableHeight, -SegmentOffsetTip + JointStandoff]])
    
    cable1.addObject('MechanicalObject', position=CablePoints.tolist())
    
    cable1.addObject('CableConstraint', template='Vec3d', name='CableConstraint', indices=list(range(2*NSegments)), pullPoint=[0, CableHeight, 0], printLog=True, value=10)                               
    cable1.addObject('BarycentricMapping')                
                                    
    # Moving Point                           
    ReferenceMONode = rootNode.addChild('ReferenceMONode')    
    ReferenceMONode.addObject("MechanicalObject", name="ReferenceMO", template="Vec3d", position=[0.0, 0, -3.0*config.Length], showObject=True, showObjectScale=10) # orientation is 240 deg away from scene origin

    #################
    # Generate Mold #
    #################
    # Generate Mold geometry only if not in an optimization loop
    if not config.in_optimization_loop:
        
        # Mold Box
        config.get_mesh_filename(mode = "Surface", refine = 1, 
                                    generating_function = MoldBox,
                 ThicknessMold = config.ThicknessMold, MoldWallThickness = config.MoldWallThickness, HeightMold = config.HeightMold, 
                 LengthMold = config.LengthMold, CableHeight = config.CableHeight, CableRadius = config.CableRadius,
                 Length = config.Length, Height = config.Height, OuterRadius = config.OuterRadius, TeethRadius = config.TeethRadius, 
                 PlateauHeight = config.PlateauHeight, JointHeight = config.JointHeight, Thickness = config.Thickness, 
                 JointSlopeAngle = config.JointSlopeAngle, FixationWidth = config.FixationWidth, BellowHeight = config.BellowHeight, 
                 NBellows = config.NBellows, WallThickness = config.WallThickness, CenterThickness = config.CenterThickness, 
                 CavityCorkThickness = config.CavityCorkThickness, lc = config.lc_finger, Stage1Mod=False)         

        # Mold Lid
        config.get_mesh_filename(mode = "Surface", refine = 1, 
                                    generating_function = MoldLid,
                 ThicknessMold = config.ThicknessMold, MoldWallThickness = config.MoldWallThickness, HeightMold = config.HeightMold, 
                 LengthMold = config.LengthMold, CableHeight = config.CableHeight, CableRadius = config.CableRadius,
                 Length = config.Length, Height = config.Height, OuterRadius = config.OuterRadius, TeethRadius = config.TeethRadius, 
                 PlateauHeight = config.PlateauHeight, JointHeight = config.JointHeight, Thickness = config.Thickness, 
                 JointSlopeAngle = config.JointSlopeAngle, FixationWidth = config.FixationWidth, BellowHeight = config.BellowHeight, 
                 NBellows = config.NBellows, WallThickness = config.WallThickness, CenterThickness = config.CenterThickness, 
                 CavityCorkThickness = config.CavityCorkThickness, lc = config.lc_finger, MoldCoverTolerance = config.MoldCoverTolerance, 
                 Stage1Mod=False)  
        

        # Cavities Cork
        config.get_mesh_filename(mode = "Surface", refine = 1, 
                                    generating_function = MoldForCork,
                    OuterRadius = config.OuterRadius, BellowHeight = config.BellowHeight, 
                    NBellows = config.NBellows, WallThickness = config.WallThickness, TeethRadius = config.TeethRadius, 
                    CenterThickness = config.CenterThickness, PlateauHeight = config.PlateauHeight, 
                    CavityCorkThickness = config.CavityCorkThickness) 

        # Finger clamp
        config.get_mesh_filename(mode = "Surface", refine = 1, 
                                    generating_function = FingerClamp,
                    MoldWallThickness = config.MoldWallThickness, LengthMold = config.LengthMold, CableHeight = config.CableHeight, CableRadius = config.CableRadius,
                 Length = config.Length, Height = config.Height, OuterRadius = config.OuterRadius, TeethRadius = config.TeethRadius, PlateauHeight = config.PlateauHeight, 
                 JointHeight = config.JointHeight, Thickness = config.Thickness, JointSlopeAngle = config.JointSlopeAngle, FixationWidth= config.FixationWidth, 
                 BellowHeight = config.BellowHeight, NBellows = config.NBellows, WallThickness = config.WallThickness, 
                 CenterThickness = config.CenterThickness, CavityCorkThickness = config.CavityCorkThickness, lc = config.lc_finger, Stage1Mod=False) 

        

    ##################
    ### Controller ###                            
    ##################
    rootNode.addObject(FitnessEvaluationController(name="FitnessEvaluationController", rootNode=rootNode, config=config))
    
    return rootNode

    
    
