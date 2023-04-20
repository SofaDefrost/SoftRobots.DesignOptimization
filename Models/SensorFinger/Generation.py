# -*- coding: utf-8 -*-
"""Shape generation for the SensorFinger"""

__authors__ = "sescaidanavarro, tnavez"
__contact__ = "stefan.escaida@uoh.cl, tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Oct 28 2022"

import gmsh
import numpy as np
import locale
locale.setlocale(locale.LC_ALL, 'en_US.UTF-8')

############################
### Basic Gmsh Functions ###
############################
def defineMeshSizes(Length, Height, Thickness, lc=0.5):   
    gmsh.model.mesh.field.add("Box", 6)
    gmsh.model.mesh.field.setNumber(6, "VIn", lc)
    gmsh.model.mesh.field.setNumber(6, "VOut", lc)
    gmsh.model.mesh.field.setNumber(6, "XMin", -Thickness)
    gmsh.model.mesh.field.setNumber(6, "XMax", Thickness)
    gmsh.model.mesh.field.setNumber(6, "YMin", 0)
    gmsh.model.mesh.field.setNumber(6, "YMax", Height)
    gmsh.model.mesh.field.setNumber(6, "ZMin", -3*Length)
    gmsh.model.mesh.field.setNumber(6, "ZMax", 0)    
    gmsh.model.mesh.field.setNumber(6, "Thickness", 0.3)

    gmsh.model.mesh.field.setAsBackgroundMesh(6)

    gmsh.option.setNumber("Mesh.CharacteristicLengthExtendFromBoundary", 0)
    gmsh.option.setNumber("Mesh.CharacteristicLengthFromPoints", 0)
    gmsh.option.setNumber("Mesh.CharacteristicLengthFromCurvature", 0)
    

def defineMeshSizesZones(Center, Height, Thickness, lc=0.5, FieldId=0):   
    gmsh.model.mesh.field.add("Box", FieldId)
    gmsh.model.mesh.field.setNumber(FieldId, "VIn", lc)
    gmsh.model.mesh.field.setNumber(FieldId, "VOut", 10)
    gmsh.model.mesh.field.setNumber(FieldId, "XMin", -Thickness)
    gmsh.model.mesh.field.setNumber(FieldId, "XMax", Thickness)
    gmsh.model.mesh.field.setNumber(FieldId, "YMin", -Height)
    gmsh.model.mesh.field.setNumber(FieldId, "YMax", Height)
    gmsh.model.mesh.field.setNumber(FieldId, "ZMin", -Center-Height)
    gmsh.model.mesh.field.setNumber(FieldId, "ZMax", -Center+ Height)    
    gmsh.model.mesh.field.setNumber(FieldId, "Thickness", 0.3)   

    gmsh.option.setNumber("Mesh.CharacteristicLengthExtendFromBoundary", 0)
    gmsh.option.setNumber("Mesh.CharacteristicLengthFromPoints", 0)
    gmsh.option.setNumber("Mesh.CharacteristicLengthFromCurvature", 0)
    
##############################
### Gmsh Utility Functions ###
##############################  
def createSegment(Length, Height, JointHeight, Thickness, JointSlopeAngle,lc=1):
        
    PointTags = np.empty((0,1), int)
    
    LengthDiagonal = (Height-JointHeight)/np.cos(JointSlopeAngle)
    JointStandoff = LengthDiagonal*np.sin(JointSlopeAngle)
    
    YValues = np.array([0,0,JointHeight,Height,Height,JointHeight])
    ZValues = np.array([0,-Length,-Length, -Length+JointStandoff,-JointStandoff,0])
    
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(Thickness/2, YValue, ZValue, lc) for (YValue,ZValue) in zip(YValues,ZValues)])
    
    LineTags = createLines(PointTags)
    WireLoop = gmsh.model.occ.addWire(LineTags)
    SurfaceTag = gmsh.model.occ.addPlaneSurface([WireLoop])
    ExtrudeTags = gmsh.model.occ.extrude([(2,SurfaceTag)],-Thickness,0,0)
    print("Segment extrude dim tags:", ExtrudeTags)
    gmsh.model.occ.synchronize()
    
    return ExtrudeTags[1]

def createLines(PointTags):
    LineTags = np.empty((0,1),dtype=int)
    NPoints = len(PointTags)
    for i in range(1, NPoints):
        LineTags = np.append(LineTags, [gmsh.model.occ.addLine(PointTags[i-1], PointTags[i])])
    LineTags = np.append(LineTags, [gmsh.model.occ.addLine(PointTags[-1], PointTags[0])])
    return LineTags 

############################################
### Finger Geometry Generation Functions ###
############################################   
def createArticulationBellow(OuterRadius, NBellows, BellowHeight, TeethRadius, FingerWidth, PlateauHeight, lc=1):
    
    PointTags = np.empty((0,1), int)
    
    DiagonalStepHeight = (BellowHeight - PlateauHeight)/2
    TotalBellowHeight = NBellows*BellowHeight
    
    XValueIn = TeethRadius     
    XValueOut = OuterRadius 
    
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(0, 0, 0, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(XValueIn, 0, 0, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(XValueOut, 0, DiagonalStepHeight, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(XValueOut, 0, DiagonalStepHeight+PlateauHeight, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(XValueIn, 0, BellowHeight, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(0, 0, BellowHeight, lc)])    

    LineTags = createLines(PointTags)
    WireLoop = gmsh.model.occ.addWire(LineTags)
    SurfaceTag = gmsh.model.occ.addPlaneSurface([WireLoop])                
    
    RevolveDimTags = gmsh.model.occ.revolve([(2,SurfaceTag)], 0,0,0, 0,0,1, np.pi)
    SingleBellowDimTag = RevolveDimTags[1]    

    Copies = []
    BellowDimTags = None
    
    if NBellows > 1:
        for i in range(1,NBellows):
            CopyDimTags = gmsh.model.occ.copy([SingleBellowDimTag])
            gmsh.model.occ.translate(CopyDimTags,0,0,i*BellowHeight)
            Copies.append(CopyDimTags[0])
        
    
        FuseOut = gmsh.model.occ.fuse([SingleBellowDimTag], Copies)
        BellowDimTags = FuseOut[0]    
    else:
        BellowDimTags = [SingleBellowDimTag]
        
    print("BellowDimTag: {}".format(BellowDimTags))
    gmsh.model.occ.translate(BellowDimTags,0,0,-TotalBellowHeight/2)
#    gmsh.model.occ.synchronize()
#    gmsh.fltk.run()
    return BellowDimTags


def createCavitySketch(OuterRadius, BellowHeight, TeethRadius, WallThickness, CenterThickness, PlateauHeight, lc=1 ):
    
    PointTags = np.empty((0,1), int)
    
    DiagonalStepHeight = (BellowHeight - PlateauHeight)/2
    
    XValueIn = TeethRadius - WallThickness
    XValueOut = OuterRadius - WallThickness
    
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(0, 0, 0, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(XValueIn, 0, 0, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(XValueOut, 0, DiagonalStepHeight, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(XValueOut, 0, DiagonalStepHeight+PlateauHeight, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(XValueIn, 0, BellowHeight, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(0, 0, BellowHeight, lc)])    

    LineTags = createLines(PointTags)
    WireLoop = gmsh.model.occ.addWire(LineTags)
    SurfaceTag = gmsh.model.occ.addPlaneSurface([WireLoop])            
    return SurfaceTag
    

def createCavityVolume(OuterRadius, NBellows, BellowHeight, TeethRadius, WallThickness, CenterThickness, CavityCorkThickness, PlateauHeight):
    
    TotalHeight = NBellows * BellowHeight
    SurfaceTag = createCavitySketch(OuterRadius, BellowHeight, TeethRadius, WallThickness, CenterThickness, PlateauHeight)
    RevolveDimTags = gmsh.model.occ.revolve([(2,SurfaceTag)], 0,0,0, 0,0,1, np.pi)
    CavityDimTag = RevolveDimTags[1]
    CavityBaseDimTag = None    
    Copies = []
    if NBellows > 1:
        for i in range(1,NBellows):        
            HalfCopyDimTag = gmsh.model.occ.copy([CavityDimTag])
            Copies.append(HalfCopyDimTag[0])
            gmsh.model.occ.translate(HalfCopyDimTag, 0,0,i*BellowHeight)         
        
        FusionOut = gmsh.model.occ.fuse([CavityDimTag], Copies)
        
        CavityBaseDimTag = FusionOut[0]
    else:
        CavityBaseDimTag = [CavityDimTag] 
    gmsh.model.occ.translate(CavityBaseDimTag,0,0,-TotalHeight/2)
    BoxDimTag = (3,gmsh.model.occ.addBox(-OuterRadius,
                                    0,
                                    -(TotalHeight+1),
                                    2*OuterRadius, 
                                    CavityCorkThickness, 
                                    2*(TotalHeight+1)))    
        
    CutOut = gmsh.model.occ.cut(CavityBaseDimTag,[BoxDimTag])
    print("CutOut: ", CutOut)
    CavityDimTags = CutOut[0]
#    gmsh.model.occ.synchronize()
#    gmsh.fltk.run()
    return CavityDimTags


def createAllCavities(Length, OuterRadius, NBellows, BellowHeight, TeethRadius, WallThickness, CenterThickness, CavityCorkThickness, PlateauHeight, lc=1):
    
    BellowGap = (NBellows-1)*BellowHeight
    Cavity1DimTags = createCavityVolume(OuterRadius, NBellows, BellowHeight, TeethRadius, WallThickness, CenterThickness, CavityCorkThickness, PlateauHeight)
    gmsh.model.occ.translate(Cavity1DimTags,0,0,-(Length+BellowGap/2))
    Cavity2DimTags = gmsh.model.occ.copy(Cavity1DimTags)        
    gmsh.model.occ.translate(Cavity2DimTags,0,0,-(Length+BellowGap))
    gmsh.model.occ.synchronize()
    AllCavitiesDimTags = Cavity1DimTags + Cavity2DimTags 
    return AllCavitiesDimTags

def createCavity(OuterRadius, NBellows, BellowHeight, TeethRadius, WallThickness, CenterThickness, CavityCorkThickness, PlateauHeight, Z_translation):
    gmsh.model.add("Cavity_" + str(Z_translation))            
    Cavity1DimTags = createCavityVolume(OuterRadius, NBellows, BellowHeight, TeethRadius, WallThickness, CenterThickness, CavityCorkThickness, PlateauHeight)
    gmsh.model.occ.translate(Cavity1DimTags,0,0,Z_translation)

def makeSegmentStage1Mod(SegmentDimTag, Length, Height, JointHeight, Thickness, JointSlopeAngle, lc=1):
        
    LengthDiagonal = (Height-JointHeight)/np.cos(JointSlopeAngle)
    JointStandoff = LengthDiagonal*np.sin(JointSlopeAngle)    
    Stage1DistanceToStandoff = 2
    Stage1DistanceFromBase = 5
    Overboarding = 3
    SubstractionDepth = 2.5
    ZOffset = JointStandoff+Stage1DistanceToStandoff
    BoxSubstractionOuterDimTag = (3,gmsh.model.occ.addBox(-Thickness/2-Overboarding, 
                                                        Stage1DistanceFromBase,
                                                        -ZOffset, 
                                                        Thickness+2*Overboarding, 
                                                        Height+Overboarding,
                                                        -(Length-2*ZOffset)))
    
    BoxSubstractionInnerDimTag = (3,gmsh.model.occ.addBox(-Thickness/2+SubstractionDepth, 
                                                        Stage1DistanceFromBase,
                                                        -ZOffset,
                                                        Thickness-2*SubstractionDepth,
                                                        Height-(SubstractionDepth+Stage1DistanceFromBase),
                                                        -(Length-2*ZOffset)))
    
    CutOut = gmsh.model.occ.cut([BoxSubstractionOuterDimTag], [BoxSubstractionInnerDimTag])
    SubstractionObjectDimTag = CutOut[0][0]
    
    CutOut = gmsh.model.occ.cut([SegmentDimTag],[SubstractionObjectDimTag])
    Segment1ModDimTag = CutOut[0][0]
    return Segment1ModDimTag
    

def makeSegmentFixationMod(SegmentDimTag, Length, Height, JointHeight, Thickness, JointSlopeAngle, FixationWidth, lc=1):

    BoxFillDimTag = gmsh.model.occ.addBox(-Thickness/2,
                                        0,
                                        0,
                                        Thickness,
                                        Height,
                                        -Length/2)
    
    BoxFixationDimTag = gmsh.model.occ.addBox(-(Thickness/2+FixationWidth),
                                            0,
                                            0,
                                            Thickness+2*FixationWidth, 
                                            Height+FixationWidth,
                                            -FixationWidth)
    
    BoxesDimTags = [(3,BoxFillDimTag),(3,BoxFixationDimTag)]
    FuseOut = gmsh.model.occ.fuse([SegmentDimTag],BoxesDimTags)
    SegmentDimTag = FuseOut[0][0]
    return SegmentDimTag

    
def createFinger(Length, Height, OuterRadius, TeethRadius, PlateauHeight, JointHeight, 
                 Thickness, JointSlopeAngle, FixationWidth, BellowHeight, NBellows, WallThickness, 
                 CenterThickness, CavityCorkThickness, lc, Stage1Mod=False):
    
    #-------------------
    # Segments 
    #-------------------            
    Segment1DimTag = createSegment(Length, Height, JointHeight, Thickness, JointSlopeAngle)   
      
    Segment2DimTags = gmsh.model.occ.copy([Segment1DimTag])
    if Stage1Mod:
        Segment2DimTags = [makeSegmentStage1Mod(Segment2DimTags[0], Length, Height, JointHeight, Thickness, JointSlopeAngle, lc=1)]    
    Segment3DimTags = gmsh.model.occ.copy(Segment2DimTags)
    Segment1DimTag = makeSegmentFixationMod(Segment1DimTag, Length, Height, JointHeight, Thickness, JointSlopeAngle, FixationWidth)    
   
    BellowGap = BellowHeight*(NBellows-1)
    gmsh.model.occ.translate(Segment2DimTags,0,0,-(BellowGap+Length))
    gmsh.model.occ.translate(Segment3DimTags,0,0,-2*(BellowGap+Length))    
    
    #-------------------
    # Bellows
    #-------------------           
    Bellow1DimTags = createArticulationBellow(OuterRadius, NBellows, BellowHeight, TeethRadius, Thickness, PlateauHeight, lc=lc)
    Bellow2DimTags = gmsh.model.occ.copy(Bellow1DimTags)
    gmsh.model.occ.translate(Bellow1DimTags,0,0,-(BellowGap/2+Length))
    gmsh.model.occ.translate(Bellow2DimTags,0,0,-2*(3*BellowGap/4+Length))
    FuseOut = gmsh.model.occ.fuse(Bellow1DimTags,[Segment1DimTag]+Segment2DimTags + Segment3DimTags + Bellow2DimTags)
    FingerNoCavitiesDimTag = FuseOut[0]
    gmsh.model.occ.synchronize()
    
    #-------------------
    # Cavities
    #-------------------
    AllCavitiesDimTags = createAllCavities(Length, OuterRadius, NBellows, BellowHeight, TeethRadius, WallThickness, CenterThickness, CavityCorkThickness, PlateauHeight, lc=lc)
    
    #-------------------
    # Cut Cavities
    #-------------------
    CutOut = gmsh.model.occ.cut(FingerNoCavitiesDimTag,AllCavitiesDimTags)
    FingerDimTag = CutOut[0][0]
    gmsh.model.occ.synchronize()

    return FingerDimTag

#####################################
### Geometry Generation Functions ###
#####################################  
def Cavity(Length, Height, Thickness, OuterRadius, NBellows, BellowHeight, TeethRadius, WallThickness, CenterThickness, CavityCorkThickness, PlateauHeight, Z_translation, RefineAroundCavities = False):
    
    # Create cavity geometry
    createCavity(OuterRadius, NBellows, BellowHeight, TeethRadius, WallThickness, CenterThickness, CavityCorkThickness, PlateauHeight, Z_translation)

    # Mesh
    if RefineAroundCavities:
        lc = 0.11
    else:
        lc = 0.5
    gmsh.model.occ.synchronize()
    defineMeshSizes(Length, Height, Thickness, lc)
    return 0


def Finger(Length, Height, OuterRadius, TeethRadius, PlateauHeight, JointHeight, 
                 Thickness, JointSlopeAngle, FixationWidth, BellowHeight, NBellows, WallThickness, 
                 CenterThickness, CavityCorkThickness, lc, Stage1Mod=False, RefineAroundCavities=False):
    
    # Create finger geometry
    FingerDimTag = createFinger(Length, Height, OuterRadius, TeethRadius, PlateauHeight, JointHeight, 
                 Thickness, JointSlopeAngle, FixationWidth, BellowHeight, NBellows, WallThickness, 
                 CenterThickness, CavityCorkThickness, lc, Stage1Mod=Stage1Mod)
    
    # Refined mesh size around cavities
    if RefineAroundCavities:
        LcLocal = 2
        defineMeshSizesZones(Length, BellowHeight/2, Thickness, lc=LcLocal, FieldId=1)
        defineMeshSizesZones(2*Length, BellowHeight/2, Thickness, lc=LcLocal, FieldId=2)

        gmsh.model.mesh.field.add("Min", 3)
        gmsh.model.mesh.field.setNumbers(3, "FieldsList", [1,2])
        gmsh.model.mesh.field.setAsBackgroundMesh(3)
    else:
        # Homogeneous mesh refinement for the finger
        defineMeshSizes(Length, Height, Thickness, lc)

    return FingerDimTag

