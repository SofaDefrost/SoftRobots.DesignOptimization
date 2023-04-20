# -*- coding: utf-8 -*-
"""Mold generation for the SensorFinger"""

__authors__ = "sescaidanavarro, tnavez"
__contact__ = "stefan.escaida@uoh.cl, tanguy.navez@inria.fr"
__version__ = "1.0.0"
__copyright__ = "(c) 2020, Inria"
__date__ = "Oct 28 2022"

import gmsh 
import numpy as np
from Generation import createFinger, createCavitySketch


def dimTagz2Tagz(DimTagz, dimension):
    Tagz = [tag for (dim, tag) in DimTagz]
    return Tagz


def hide_all():
    ent = gmsh.model.getEntities()
    for x in ent:
        gmsh.model.setVisibility((x,), False)

def createFingerMold(ThicknessMold, MoldWallThickness, HeightMold, LengthMold, CableHeight, CableRadius,
                 Length, Height, OuterRadius, TeethRadius, PlateauHeight, JointHeight, 
                 Thickness, JointSlopeAngle, FixationWidth, BellowHeight, NBellows, WallThickness, 
                 CenterThickness, CavityCorkThickness, lc, Stage1Mod=False):   
       
    FingerDimTag = createFinger(Length, Height, OuterRadius, TeethRadius, PlateauHeight, JointHeight, 
                 Thickness, JointSlopeAngle, FixationWidth, BellowHeight, NBellows, WallThickness, 
                 CenterThickness, CavityCorkThickness, lc, Stage1Mod=Stage1Mod)


    MoldBoxDimTag = (3,gmsh.model.occ.addBox(-ThicknessMold/2,
                                            0,
                                            MoldWallThickness, 
                                            ThicknessMold, 
                                            HeightMold, 
                                            -LengthMold))

    CableLength = LengthMold+2*MoldWallThickness
    CableDimTag = (3,gmsh.model.occ.addCylinder(0,CableHeight,2*MoldWallThickness,0,0,-CableLength,CableRadius))
    CutOut = gmsh.model.occ.cut([MoldBoxDimTag],[FingerDimTag, CableDimTag])
    MoldBaseDimTag = CutOut[0][0]
    AllCavitiesDimTags = CutOut[0][1:]
    MoldBoxOuterRimDimTag = (3,gmsh.model.occ.addBox(-ThicknessMold/2,
                                                    0,
                                                    MoldWallThickness, 
                                                    ThicknessMold, 
                                                    -MoldWallThickness, 
                                                    -LengthMold))
    
    MoldBoxInnerRimDimTag = (3,gmsh.model.occ.addBox(-ThicknessMold/2+MoldWallThickness,
                                                    0,
                                                    0,
                                                    ThicknessMold-2*MoldWallThickness,
                                                    -MoldWallThickness, 
                                                    -LengthMold+2*MoldWallThickness))
    CutOut = gmsh.model.occ.cut([MoldBoxOuterRimDimTag],[MoldBoxInnerRimDimTag])
    MoldRim = CutOut[0][0]
    FuseOut = gmsh.model.occ.fuse([MoldBaseDimTag],[MoldRim])
    MoldDimTag = FuseOut[0][0]
    
    gmsh.model.occ.synchronize()

    return MoldDimTag, AllCavitiesDimTags



def createMoldLid(Length, OuterRadius, BellowHeight, WallThickness, TeethRadius, CenterThickness, PlateauHeight,
                    CavityCorkThickness, ThicknessMold, MoldWallThickness, LengthMold, 
                    MoldCoverTolerance, AllCavitiesDimTags):
    
    #-----------------
    # Create mold lid
    #-----------------
    MoldLidTopDimTag = (3,gmsh.model.occ.addBox(-ThicknessMold/2,
                                                -MoldWallThickness,
                                                MoldWallThickness, 
                                                ThicknessMold, 
                                                -MoldWallThickness, 
                                                -LengthMold))
    gmsh.model.occ.synchronize()

    SurfaceBorderDimTags = gmsh.model.getBoundary([MoldLidTopDimTag],oriented=False)     
    LineBorderDimTags = gmsh.model.getBoundary(SurfaceBorderDimTags[0:], combined=False, oriented=False)
    BorderTagz = dimTagz2Tagz(LineBorderDimTags,1)
    gmsh.model.occ.fillet([MoldLidTopDimTag[1]], BorderTagz, [0.7])
    gmsh.model.occ.synchronize()

    MoldLidInteriorDimTag = (3,gmsh.model.occ.addBox(-ThicknessMold/2+MoldWallThickness+MoldCoverTolerance,
                                                    0,
                                                    MoldCoverTolerance, 
                                                    ThicknessMold-2*MoldWallThickness-2*MoldCoverTolerance,
                                                    -MoldWallThickness, 
                                                    -LengthMold+2*MoldWallThickness+2*MoldCoverTolerance))
    
    #-----------------
    # Create cavity cork
    #-----------------
    CorkBellowHeight = BellowHeight+2
    CorkWallThickness = WallThickness-1
    CavityCorkSketchDimTag = (2, createCavitySketch(OuterRadius, CorkBellowHeight, TeethRadius, CorkWallThickness, CenterThickness, PlateauHeight))
    ExtrudeDimTags = gmsh.model.occ.extrude([CavityCorkSketchDimTag],0,CavityCorkThickness,0)
    HalfDimTag = ExtrudeDimTags[1]
    CavityCorkDimTags = [HalfDimTag]
    
    gmsh.model.occ.translate(CavityCorkDimTags,0,0,-Length-CorkBellowHeight/2)
    CavityCork2DimTags = gmsh.model.occ.copy(CavityCorkDimTags)
    CavityCork3DimTags = gmsh.model.occ.copy(CavityCorkDimTags)
    CavityCork4DimTags = gmsh.model.occ.copy(CavityCorkDimTags)    
    
    gmsh.model.occ.affineTransform(CavityCork2DimTags, [-1,0,0,0, 0,1,0,0, 0,0,1,0])
    gmsh.model.occ.translate(CavityCork3DimTags,0,0,-Length)
    gmsh.model.occ.affineTransform(CavityCork4DimTags, [-1,0,0,0, 0,1,0,0, 0,0,1,0])
    gmsh.model.occ.translate(CavityCork4DimTags,0,0,-Length)
    gmsh.model.occ.synchronize()
    AllCavitiesCorkDimTags = CavityCorkDimTags + CavityCork2DimTags + CavityCork3DimTags + CavityCork4DimTags
    
    FuseOut = gmsh.model.occ.fuse([MoldLidTopDimTag],[MoldLidInteriorDimTag]+AllCavitiesCorkDimTags+AllCavitiesDimTags)
    LidDimTag = FuseOut[0][0]
            
    return LidDimTag


#################################
### Mesh Generation Functions ###
################################# 
def MoldBox(ThicknessMold, MoldWallThickness, HeightMold, LengthMold, CableHeight, CableRadius,
                 Length, Height, OuterRadius, TeethRadius, PlateauHeight, JointHeight, 
                 Thickness, JointSlopeAngle, FixationWidth, BellowHeight, NBellows, WallThickness, 
                 CenterThickness, CavityCorkThickness, lc, Stage1Mod=False):

    MoldDimTag, AllCavitiesDimTags = createFingerMold(ThicknessMold, MoldWallThickness, HeightMold, LengthMold, CableHeight, CableRadius,
                 Length, Height, OuterRadius, TeethRadius, PlateauHeight, JointHeight, 
                 Thickness, JointSlopeAngle, FixationWidth, BellowHeight, NBellows, WallThickness, 
                 CenterThickness, CavityCorkThickness, lc, Stage1Mod=Stage1Mod)

    # Remove cavities
    CutOut = gmsh.model.occ.cut([MoldDimTag], AllCavitiesDimTags)

    return MoldDimTag


def MoldLid(ThicknessMold, MoldWallThickness, HeightMold, LengthMold, CableHeight, CableRadius,
                 Length, Height, OuterRadius, TeethRadius, PlateauHeight, JointHeight, 
                 Thickness, JointSlopeAngle, FixationWidth, BellowHeight, NBellows, WallThickness, 
                 CenterThickness, CavityCorkThickness, lc, MoldCoverTolerance, Stage1Mod=False):
    
    MoldDimTag, AllCavitiesDimTags = createFingerMold(ThicknessMold, MoldWallThickness, HeightMold, LengthMold, CableHeight, CableRadius,
                 Length, Height, OuterRadius, TeethRadius, PlateauHeight, JointHeight, 
                 Thickness, JointSlopeAngle, FixationWidth, BellowHeight, NBellows, WallThickness, 
                 CenterThickness, CavityCorkThickness, lc, Stage1Mod=Stage1Mod)
    
    LidDimTag = createMoldLid(Length, OuterRadius, BellowHeight, WallThickness, TeethRadius, CenterThickness, PlateauHeight,
                    CavityCorkThickness, ThicknessMold, MoldWallThickness, LengthMold, 
                    MoldCoverTolerance, AllCavitiesDimTags)
    
    # hide_all(gmsh)
    # gmsh.model.setVisibility((LidDimTag,),False, True)
    # gmsh.model.setVisibility((MoldDimTag,),False, True)  

    # Remove Box
    CutOut = gmsh.model.occ.cut([LidDimTag], [MoldDimTag])

    return LidDimTag


def MoldForCork(OuterRadius, BellowHeight, NBellows, WallThickness, TeethRadius, 
                       CenterThickness, PlateauHeight, CavityCorkThickness):
    
    CorkBellowHeight = BellowHeight+2
    CorkWallThickness = WallThickness-1
    CavityCorkSketchDimTag = (2, createCavitySketch(OuterRadius, CorkBellowHeight, TeethRadius, CorkWallThickness, CenterThickness, PlateauHeight))

    Tolerance = 2
    TotalBellowHeight = NBellows * BellowHeight
    CorkMoldHeight = TotalBellowHeight + 2 * Tolerance
    CorkMoldThickness = (OuterRadius+Tolerance) * 2    
    ExtrudeDimTags = gmsh.model.occ.extrude([CavityCorkSketchDimTag],0,CavityCorkThickness,0)    
    HalfDimTag = ExtrudeDimTags[1]

    CavityCorkDimTag = HalfDimTag
    gmsh.model.occ.translate([CavityCorkDimTag],0,0,-TotalBellowHeight/2)
    
    CavityCork2DimTags = gmsh.model.occ.copy([CavityCorkDimTag])
    gmsh.model.occ.affineTransform(CavityCork2DimTags, [-1,0,0,0, 0,1,0,0, 0,0,1,0])
    
    FuseOut = gmsh.model.occ.fuse([CavityCorkDimTag],CavityCork2DimTags)
    
    CompleteCorkDimTag = FuseOut[0][0]
    CavityMoldBoxDimTag = (3,gmsh.model.occ.addBox(-CorkMoldThickness/2,
                                                -Tolerance,
                                                -CorkMoldHeight/2,
                                                CorkMoldThickness, 
                                                CavityCorkThickness+Tolerance,
                                                CorkMoldHeight+Tolerance))
    CutOut = gmsh.model.occ.cut([CavityMoldBoxDimTag],[CompleteCorkDimTag])

    return CutOut
    
    
def FingerClamp(MoldWallThickness, LengthMold, CableHeight, CableRadius,
                 Length, Height, OuterRadius, TeethRadius, PlateauHeight, JointHeight, 
                 Thickness, JointSlopeAngle, FixationWidth, BellowHeight, NBellows, WallThickness, 
                 CenterThickness, CavityCorkThickness, lc, Stage1Mod=False):
    
    FingerDimTags = [createFinger(Length, Height, OuterRadius, TeethRadius, PlateauHeight, JointHeight, 
                 Thickness, JointSlopeAngle, FixationWidth, BellowHeight, NBellows, WallThickness, 
                 CenterThickness, CavityCorkThickness, lc, Stage1Mod=Stage1Mod)]
    
    ClampBoxWidth = 4*FixationWidth+Thickness
    ClampBoxLength = 4*FixationWidth
    ClampBoxHeight = Height + 2 * FixationWidth 
    ClampBoxDimDag = (3,gmsh.model.occ.addBox(-ClampBoxWidth/2,
                                        0,
                                        -ClampBoxLength/2,
                                        ClampBoxWidth,
                                        ClampBoxHeight,
                                        ClampBoxLength))
    ClampBoxCablePassDimTag = (3,gmsh.model.occ.addBox(-Thickness/2, 
                                                    0,
                                                    0,
                                                    Thickness,
                                                    5,
                                                    10
                                                    ))
     
    ScrewRadius = 1.7
    ScrewEarWidth = 6
    ScrewEarHeight = 3 
    ScrewEarLength = ScrewEarWidth
    
    ScrewEarBoxDimDag = (3,gmsh.model.occ.addBox(ClampBoxWidth/2,
                                        0,
                                        -ScrewEarLength/2,
                                        ScrewEarWidth,
                                        ScrewEarHeight,
                                        ScrewEarLength))
    
    ScrewLength = 6
    ScrewCylinderDimTag = (3,gmsh.model.occ.addCylinder(ClampBoxWidth/2+ScrewEarWidth/2,-ScrewLength/3,0, 0,ScrewLength,0,ScrewRadius))
    
    
    ScrewEarBox2DimTags = gmsh.model.occ.copy([ScrewEarBoxDimDag])
    gmsh.model.occ.affineTransform(ScrewEarBox2DimTags, [-1,0,0,0, 0,1,0,0, 0,0,1,0])
    
    ScrewCylinder2DimTags = gmsh.model.occ.copy([ScrewCylinderDimTag])
    gmsh.model.occ.affineTransform(ScrewCylinder2DimTags, [-1,0,0,0, 0,1,0,0, 0,0,1,0])
    
    CableLength = LengthMold+2*MoldWallThickness
    CableDimTag = (3,gmsh.model.occ.addCylinder(0,CableHeight,2*MoldWallThickness,0,0,-CableLength,CableRadius))
    
    FuseOut = gmsh.model.occ.fuse([ClampBoxDimDag],[ScrewEarBoxDimDag]+ScrewEarBox2DimTags)
    PositiveBoxDimTag = FuseOut[0][0]
    
    gmsh.model.occ.cut([PositiveBoxDimTag],FingerDimTags+ScrewCylinder2DimTags + [ScrewCylinderDimTag,CableDimTag]+[ClampBoxCablePassDimTag])
    
    return 0



