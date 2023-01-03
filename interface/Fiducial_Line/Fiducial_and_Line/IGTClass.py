import slicer, qt, ctk
from Switch import PyToogle

class IGT(): 
    def __init__(self, layout):
        self.layout = layout
    # NODE 
        self.IGTNode = slicer.vtkMRMLIGTLConnectorNode()
        self.IGTNode.SetName('IGT Connector')
        slicer.mrmlScene.AddNode(self.IGTNode)
        self.IGTNode.SetTypeServer(18944)
        self.IGTNode.SetTypeClient('localhost', 18944)
    
    def ResponseSwitch(self):        
        if self.IGTSwitch.isChecked():
            self.IGTNode.Start()
        else:
            self.IGTNode.Stop()

    def SetupIGT(self):
        font = qt.QFont()
        font.setPixelSize(14)
        # IGT AREA
        self.IGTCollapsibleButton = ctk.ctkCollapsibleButton()
        self.IGTCollapsibleButton.text = "IGT connection"
         # LABEL
        self.layout.addWidget(self.IGTCollapsibleButton)
        IGTLayout = qt.QFormLayout(self.IGTCollapsibleButton)
        self.IGTLabelSwitch = qt.QLabel("IGT connection")
        self.IGTLabelSwitch.setFont(font)
    
        # BUTTON
        self.IGTSwitch = PyToogle()
        IGTLayout.addRow(self.IGTLabelSwitch, self.IGTSwitch)
    
        # CONNECTION
        self.IGTSwitch.stateChanged.connect(self.ResponseSwitch)