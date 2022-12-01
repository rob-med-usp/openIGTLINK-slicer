import os
import numpy as np
import slicer, ctk, vtk, qt
import sys

class LineNode():
    def __init__(self, name, id):
    
    # LINE MARKUPS
        self.Line = slicer.vtkMRMLMarkupsLineNode()
        slicer.mrmlScene.AddNode(self.Line)
        self.Line.SetName(f"{name}")
        self.Line.GetDisplayNode().SetVisibility(True)

    # FIDUCIAL MARKUPS
        self.Contacts = slicer.vtkMRMLMarkupsFiducialNode()
        slicer.mrmlScene.AddNode(self.Contacts)
        self.Contacts.GetDisplayNode().SetTextScale(0)
        self.Contacts.GetDisplayNode().SetGlyphScale(1.5)
        self.Contacts.GetDisplayNode().SetColor(255,0,4)
        self.Contacts.GetDisplayNode().SetVisibility(False)
        self.creation = True
        

    def Observer(self):
        self.Line.AddObserver(slicer.vtkMRMLMarkupsNode.PointEndInteractionEvent, self.onMarkupEndInteraction)
        
    def onMarkupEndInteraction(self, caller, event):
        markupsNode = caller
        sliceView = markupsNode.GetAttribute("Markups.MovingInSliceView")
        movingMarkupIndex = markupsNode.GetDisplayNode().GetActiveControlPoint()
        self.Line.AddObserver(slicer.vtkMRMLMarkupsNode.PointModifiedEvent, self.onMarkupChanged)
        self.ElectrodeContact()
        
        
    def onMarkupChanged(self, caller, event):
        markupsNode = caller
        sliceView = markupsNode.GetAttribute("Markups.MovingInSliceView")
        movingMarkupIndex = markupsNode.GetDisplayNode().GetActiveControlPoint()
        self.ElectrodeContact()
             
            
    def Add(self):
        interactionNode = slicer.app.applicationLogic().GetInteractionNode()
        Node = slicer.app.applicationLogic().GetSelectionNode()
        Node.SetReferenceActivePlaceNodeClassName("vtkMRMLMarkupsLineNode")
        interactionNode.SetCurrentInteractionMode(interactionNode.Place)
    
    def DeleteNode(self):
        slicer.mrmlScene.RemoveNode(self.Line)
        slicer.mrmlScene.RemoveNode(self.Contacts)

    def Apply(self):
        self.FiducialCreation()
        self.ElectrodeContact()
      
    def Rename(self, name):
        self.Line.SetName(name)

    def vectorLine(self):
       self.line = slicer.util.arrayFromMarkupsControlPoints(self.Line)
       self.RegistrationArray = np.array([self.line[0], self.line[1]])
       print(self.RegistrationArray)
       Distance = np.linalg.norm((self.line[0]-self.line[1]))
       versor = (self.line[1]-self.line[0])/Distance
       return versor, Distance
    
    def FiducialCreation(self):
        if self.creation:
            position = [0,0,0]
            for elements in range (9):
                self.Contacts.AddControlPoint(position)
            
            self.creation = False
         
    def ElectrodeContact(self):
        self.Contacts.GetDisplayNode().SetVisibility(True)
        versor, distance = self.vectorLine() 
        self.factor = 10
        spacing = distance/self.factor 
        step = 1
        position = spacing*step
        for step in range(self.factor-1):
            position = self.line[0] + versor*spacing*(step+1)
            self.Contacts.SetNthFiducialPositionFromArray(step, position)

    def ViewOff(self):
        self.Line.GetDisplayNode().SetVisibility(False)

    def ViewOn(self):
        self.Line.GetDisplayNode().SetVisibility(True)