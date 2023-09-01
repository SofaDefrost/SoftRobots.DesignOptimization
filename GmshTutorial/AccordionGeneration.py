#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Dec 15 14:56:53 2020

@author: stefan
"""
import gmsh
import numpy as np

def setMeshingOptions(Width, Depth, Height):
    lc=4
    gmsh.model.mesh.field.add("Box", 6)
    gmsh.model.mesh.field.setNumber(6, "VIn", lc)
    gmsh.model.mesh.field.setNumber(6, "VOut", lc)
    gmsh.model.mesh.field.setNumber(6, "XMin", 0)
    gmsh.model.mesh.field.setNumber(6, "XMax", Width)
    gmsh.model.mesh.field.setNumber(6, "YMin", 0)
    gmsh.model.mesh.field.setNumber(6, "YMax", Depth)
    gmsh.model.mesh.field.setNumber(6, "ZMin", 0)
    gmsh.model.mesh.field.setNumber(6, "ZMax", Height)    
    gmsh.model.mesh.field.setNumber(6, "Thickness", 3)
         
    gmsh.model.mesh.field.setAsBackgroundMesh(6)
    
    gmsh.option.setNumber("Mesh.CharacteristicLengthExtendFromBoundary", 0)
    gmsh.option.setNumber("Mesh.CharacteristicLengthFromPoints", 0)
    gmsh.option.setNumber("Mesh.CharacteristicLengthFromCurvature", 0)
    

def createLines(PointTags):
    
    LineTags = np.empty((0,1),dtype=int)
    NPoints = len(PointTags)
    for i in range(1, NPoints):
        LineTags = np.append(LineTags, [gmsh.model.occ.addLine(PointTags[i-1], PointTags[i])])
    
    LineTags = np.append(LineTags, [gmsh.model.occ.addLine(PointTags[-1], PointTags[0])])
    
    #print('LineTags: ' + str(LineTags))
    return LineTags  

def createCavity(Radius, NSegments, SegmentHeight, TeethDepth, WallThickness, lc=1):    
    
    
    PointTags = np.empty((0,1), int)
    TotalHeight = SegmentHeight*NSegments
    
    YValues = np.linspace(0, TotalHeight, NSegments*2+1)
    XValues = np.ones(len(YValues)) 
    XValues[0::2] = Radius
    XValues[1::2] = Radius-TeethDepth
    XValues = XValues - WallThickness

    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(0,0,0, lc)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(XValue, YValue, 0, lc) for (XValue,YValue) in zip(XValues,YValues)])
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(0,TotalHeight,0, lc)])
   
    LineTags = createLines(PointTags)
    WireLoop = gmsh.model.occ.addWire(LineTags)
    SurfaceTag = gmsh.model.occ.addPlaneSurface([WireLoop])
    
    RevolveDimTags = gmsh.model.occ.revolve([(2,SurfaceTag)], 0,0,0, 0,1,0, np.pi)
    HalfDimTag = RevolveDimTags[1]
    
    HalfCopyDimTags = gmsh.model.occ.copy([HalfDimTag])
    gmsh.model.occ.affineTransform(HalfCopyDimTags, [1,0,0,0, 0,1,0,0, 0,0,-1,0])
 
    FusionOut = gmsh.model.occ.fuse([HalfDimTag], HalfCopyDimTags)
    CavityDimTags = FusionOut[0]
        
    return CavityDimTags 
    
    

def createAccordion(Radius, NSegments, SegmentHeight, TeethDepth, WallThickness, lc=1, Step=1):
    
    PointTags = np.empty((0,1), int)
    TotalHeight = SegmentHeight*NSegments
    
    YValues = np.linspace(0, TotalHeight, NSegments*2+1)
    XValues = np.ones(len(YValues)) 
    XValues[0::2] = Radius
    XValues[1::2] = Radius-TeethDepth
    
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(XValue, YValue, 0, lc) for (XValue,YValue) in zip(XValues,YValues)])
    
    if Step==1:
        gmsh.model.occ.synchronize()
        gmsh.fltk.run()

    XValues = XValues - WallThickness
    PointTags = np.append(PointTags, [gmsh.model.occ.addPoint(XValue, YValue, 0, lc) for (XValue,YValue) in zip(XValues[-1::-1],YValues[-1::-1])])
    
    
    if Step==2:
        print("PointTags: " + str(PointTags))
        gmsh.model.occ.synchronize()
        gmsh.fltk.run()
    
    LineTags = createLines(PointTags)
    WireLoop = gmsh.model.occ.addWire(LineTags)
    SurfaceTag = gmsh.model.occ.addPlaneSurface([WireLoop])
    
    if Step==3:
        gmsh.model.occ.synchronize()
        gmsh.fltk.run()

    RevolveDimTags = gmsh.model.occ.revolve([(2,SurfaceTag)], 0,0,0, 0,1,0, np.pi)
    HalfDimTag = RevolveDimTags[1]
    
    
    if Step==4:
        print("RevolveDimTags: " + str(RevolveDimTags))
        gmsh.model.occ.synchronize()
        gmsh.fltk.run()

    HalfCopyDimTag = gmsh.model.occ.copy([HalfDimTag])
    print("HalfCopyDimTag: ", HalfCopyDimTag )
    gmsh.model.occ.affineTransform(HalfCopyDimTag, [1,0,0,0, 0,1,0,0, 0,0,-1,0])
 
    FusionOut = gmsh.model.occ.fuse([HalfDimTag], HalfCopyDimTag)
    AccordionDimTags = FusionOut[0]
    
    if Step==5:
        gmsh.model.occ.synchronize()
        gmsh.fltk.run()
        
    return AccordionDimTags

def generateGeometry(Step):
    gmsh.initialize()
    gmsh.option.setNumber("General.Terminal", 1)
    
    gmsh.model.add("Accordion")
    gmsh.logger.start() 
    
    
    Radius = 9
    NSegments = 6
    SegmentHeight = 5
    TeethDepth = 3
    WallThickness = 4
    
    AccordionDimTags = createAccordion(Radius, NSegments, SegmentHeight, TeethDepth, WallThickness, lc=3, Step=Step)
    
    TotalHeight = NSegments * SegmentHeight
    
    SphereTag = gmsh.model.occ.addSphere(0,TotalHeight,0, Radius*1.4)
    SphereDimTags = [(3,SphereTag)]
    
    CylinderTag = gmsh.model.occ.addCylinder(0,0,0, 0,-SegmentHeight,0, Radius*1.2)
    CylinderDimTags = [(3,CylinderTag)]
    
    FuseOut = gmsh.model.occ.fuse(AccordionDimTags, SphereDimTags+CylinderDimTags)
       
    if Step==6:
        gmsh.model.occ.synchronize()
        gmsh.fltk.run()
    
        gmsh.model.occ.synchronize()
#    gmsh.model.occ.synchronize()
    
    # Generate volumetric and surface meshes for the whole object
    gmsh.model.occ.synchronize()
    gmsh.write('Accordion_Parametric.step')
    
    
    setMeshingOptions(Radius, Radius, TotalHeight)
    
    gmsh.model.mesh.generate(2)
    gmsh.write('Accordion_Surface.stl')
    
    if Step==7 or Step==0:
        gmsh.model.occ.synchronize()
        gmsh.fltk.run()
        return
     
    gmsh.model.mesh.generate(3)
    gmsh.model.occ.synchronize()
    gmsh.write('Accordion_Volumetric.vtk')
#   
    if Step==8:
        gmsh.model.occ.synchronize()
        gmsh.fltk.run()                
    
    gmsh.clear()

    CavityDimTags = createCavity(Radius, NSegments, SegmentHeight, TeethDepth, WallThickness, lc=3)
    SphereTag = gmsh.model.occ.addSphere(0,TotalHeight,0, Radius*1.4)
    SphereDimTags = [(3,SphereTag)]
    
    CutOut = gmsh.model.occ.cut(CavityDimTags, SphereDimTags)
    
    gmsh.model.mesh.generate(2)
    gmsh.model.occ.synchronize()
    gmsh.write('Accordion_Cavity.stl')    
    
    if Step==9:
        gmsh.model.occ.synchronize()
        gmsh.fltk.run()
    
        gmsh.model.occ.synchronize()    
        
    gmsh.model.occ.synchronize()

Step=7
print("Showing Step: " + str(Step))
generateGeometry(Step)
    