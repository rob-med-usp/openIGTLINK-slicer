import os
import sys
import slicer, qt
import ctk

# Local modules
sys.path.insert(0, os.path.dirname(__file__))
from lineNode import LineNode
from drawableButton import Button
from Switch import PyToogle
from IGTClass import IGT

class MarkupClass():
    def __init__(self, layout): 
        self.layout = layout
        self.Lines = []
        self.NameLine = []
        self.confirm = False

 
    def selectionchange(self):
        if len(self.NameLine)>0:
            select = slicer.mrmlScene.GetNodeByID("vtkMRMLSelectionNodeSingleton")
            self.id = self.SelectionNode.currentIndex
            select.SetActivePlaceNodeID(slicer.util.getNode(self.NameLine[self.id]).GetID())
            if self.confirm:
                self.node.Observer()
            self.ResponseSwitchLine()
            self.LineName.setText(self.NameLine[self.id])
        else:
            self.id = self.SelectionNode.currentIndex
            
    def ResponseSwitchLine(self):
        if self.SwitchLine.isChecked():
            self.LabelLine.setText('Exclusive Line')
            for idx in range(len(self.Lines)):
                self.Lines[idx].ViewOff()
            self.Lines[self.id].ViewOn()
        else:
            self.LabelLine.setText('All Lines')
            for elements in self.Lines:
                elements.ViewOn()

    def nameTest(self, idx):
        
        if self.LineName.text != 'Line Name' and self.LineName.text != '':
            name = self.LineName.text
        
        else:
            name = f'Line{idx+1}'
        return name

    def AddResponse(self):
        self.LineName.setText('Line Name')
        idx = len(self.Lines)
        self.confirm = True
        name = self.nameTest(idx)
        self.Lines.append(LineNode(name, idx))
        self.NameLine.append(name)
        self.node = self.Lines[idx]
        self.node.Add()
        self.SelectionNode.addItem(name)
        self.SelectionNode.setCurrentIndex(idx)
       

    def ApplyResponse(self):
        self.node = self.Lines[self.id]
        self.node.Apply()
        
        
    def DeleteResponse(self):
        self.node = self.Lines[self.id]
        self.node.DeleteNode()
        del self.NameLine[self.id]
        del self.Lines[self.id]
        self.SelectionNode.removeItem(self.id)
    
    def RenameResponse(self):
        name = self.nameTest(self.id)
        self.node.Rename(name)
        self.SelectionNode.setItemText(self.id, name)

    def selectAll(self):
         self.LineName.selectAll()


    def setupMarkup(self):
    # IGT
        self.IGT = IGT(self.layout) 
        self.IGT.SetupIGT()
    # FONT
        font = qt.QFont()
        font.setPixelSize(14) 

    # SELECTION NODE LINE
        self.SelectionNode = qt.QComboBox()
        self.SelectionNode.addItems(self.NameLine)
        self.SelectionNode.currentIndexChanged.connect(self.selectionchange)
        SelectionNodeLayout = qt.QFormLayout()
        SelecLineLabel = qt.QLabel("Selection Line")
        SelecLineLabel.setFont(font)
        SelectionNodeLayout.addRow(SelecLineLabel, self.SelectionNode)

        MarkupsCollapsibleButton = ctk.ctkCollapsibleButton()
        MarkupsCollapsibleButton.text = "Fiducial and Line"
        self.layout.addWidget(MarkupsCollapsibleButton)
        self.SwitchLine = PyToogle()
        self.LabelLine = qt.QLabel("All Lines")
        self.LabelLine.setFont(font)
        SwitchLayout = qt.QGridLayout()
        SwitchLayout.addWidget(self.LabelLine, 0, 0)
        SwitchLayout.addWidget(self.SwitchLine, 0, 2)
        self.SwitchLine.stateChanged.connect(self.ResponseSwitchLine)

        self.LineName = qt.QLineEdit('Line Name')
        self.LineName.setAlignment(qt.Qt.AlignCenter)
      

    # BUTTON CLASS
    
        ButtonsList = [Button('Add'), Button('Delete'), Button ('Apply'), Button('Rename')]

        # BUTTON LAYOUT
        AddDeleteButton = qt.QHBoxLayout()
        for element in range(len(ButtonsList)-1):
            AddDeleteButton.addWidget(ButtonsList[element])
       
        # AddDeleteButton = qt.QGridLayout()
        # AddDeleteButton.addWidget(ButtonsList[0],0,0)
        # AddDeleteButton.addWidget(ButtonsList[1],0,1)
        # AddDeleteButton.addWidget(ButtonsList[2],1,0)
        
        # BUTTON CONNECTION
        ButtonsList[0].connect('clicked()', self.AddResponse)
        ButtonsList[1].connect('clicked()', self.DeleteResponse)
        ButtonsList[2].connect('clicked()', self.ApplyResponse)
        ButtonsList[3].connect('clicked()', self.RenameResponse)

        nameLayout = qt.QGridLayout()
        nameLayout.addWidget(self.LineName, 0, 0, 1, 1)
        nameLayout.addWidget(ButtonsList[3], 0, 2)

    
    # LAYOUT
        WidgetList = [SwitchLayout, SelectionNodeLayout, AddDeleteButton, nameLayout]
        MarkupLayout = qt.QVBoxLayout(MarkupsCollapsibleButton)
        for element in WidgetList:
            MarkupLayout.addLayout(element)