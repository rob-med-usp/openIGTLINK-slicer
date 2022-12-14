import logging
import os
import numpy as np
import slicer, ctk, vtk, qt
from slicer.ScriptedLoadableModule import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import sys

#
# fist_Interface
#

class fist_Interface(ScriptedLoadableModule):
    """Uses ScriptedLoadableModule base class, available at:
    https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = "fist_Interface"  # TODO: make this more human readable by adding spaces
        self.parent.categories = ["Examples"]  # TODO: set categories (folders where the module shows up in the module selector)
        self.parent.dependencies = []  # TODO: add here list of module names that this module requires
        self.parent.contributors = ["John Doe (AnyWare Corp.)"]  # TODO: replace with "Firstname Lastname (Organization)"
        # TODO: update with short description of the module and a link to online module documentation
        self.parent.helpText = """
This is an example of scripted loadable module bundled in an extension.
See more information in <a href="https://github.com/organization/projectname#fist_Interface">module documentation</a>.
"""
        # TODO: replace with organization, grant and thanks
        self.parent.acknowledgementText = """
This file was originally developed by Jean-Christophe Fillion-Robin, Kitware Inc., Andras Lasso, PerkLab,
and Steve Pieper, Isomics, Inc. and was partially funded by NIH grant 3P41RR013218-12S1.
"""

        # Additional initialization step after application startup is complete
        slicer.app.connect("startupCompleted()", registerSampleData)


#
# Register sample data sets in Sample Data module
#

def registerSampleData():
    """
    Add data sets to Sample Data module.
    """
    # It is always recommended to provide sample data for users to make it easy to try the module,
    # but if no sample data is available then this method (and associated startupCompeted signal connection) can be removed.

    import SampleData
    iconsPath = os.path.join(os.path.dirname(__file__), 'Resources/Icons')

    # To ensure that the source code repository remains small (can be downloaded and installed quickly)
    # it is recommended to store data sets that are larger than a few MB in a Github release.

    # fist_Interface1
    SampleData.SampleDataLogic.registerCustomSampleDataSource(
        # Category and sample name displayed in Sample Data module
        category='fist_Interface',
        sampleName='fist_Interface1',
        # Thumbnail should have size of approximately 260x280 pixels and stored in Resources/Icons folder.
        # It can be created by Screen Capture module, "Capture all views" option enabled, "Number of images" set to "Single".
        thumbnailFileName=os.path.join(iconsPath, 'fist_Interface1.png'),
        # Download URL and target file name
        uris="https://github.com/Slicer/SlicerTestingData/releases/download/SHA256/998cb522173839c78657f4bc0ea907cea09fd04e44601f17c82ea27927937b95",
        fileNames='fist_Interface1.nrrd',
        # Checksum to ensure file integrity. Can be computed by this command:
        #  import hashlib; print(hashlib.sha256(open(filename, "rb").read()).hexdigest())
        checksums='SHA256:998cb522173839c78657f4bc0ea907cea09fd04e44601f17c82ea27927937b95',
        # This node name will be used when the data set is loaded
        nodeNames='fist_Interface1'
    )

    # fist_Interface2
    SampleData.SampleDataLogic.registerCustomSampleDataSource(
        # Category and sample name displayed in Sample Data module
        category='fist_Interface',
        sampleName='fist_Interface2',
        thumbnailFileName=os.path.join(iconsPath, 'fist_Interface2.png'),
        # Download URL and target file name
        uris="https://github.com/Slicer/SlicerTestingData/releases/download/SHA256/1a64f3f422eb3d1c9b093d1a18da354b13bcf307907c66317e2463ee530b7a97",
        fileNames='fist_Interface2.nrrd',
        checksums='SHA256:1a64f3f422eb3d1c9b093d1a18da354b13bcf307907c66317e2463ee530b7a97',
        # This node name will be used when the data set is loaded
        nodeNames='fist_Interface2'
    )


#
# fist_InterfaceWidget
#

class fist_InterfaceWidget(ScriptedLoadableModuleWidget):
    """Uses ScriptedLoadableModuleWidget base class, available at:
    https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
    """
    def __init__(self, parent=None):
        
      
       # ScriptedLoadableModuleWidget.__init__(self)
       
    #     """If parent widget is not specified: a top-level widget is created automatically;
    #     the application has to delete this widget (by calling widget.parent.deleteLater() to avoid memory leaks.
    #     """ 
    #     # Get module name by stripping 'Widget' from the class name
        self.moduleName = self.__class__.__name__
        if self.moduleName.endswith('Widget'):
            self.moduleName = self.moduleName[:-6]
        self.developerMode = slicer.util.settingsValue('Developer/DeveloperMode', True, converter=slicer.util.toBool)
        if not parent:
            self.parent = slicer.qMRMLWidget()
            self.parent.setLayout(qt.QVBoxLayout())
            self.parent.setMRMLScene(slicer.mrmlScene)
        else:
            self.parent = parent
        self.layout = self.parent.layout()
        if not parent:
            self.setup()
            self.parent.show()
        slicer.app.moduleManager().connect(
            'moduleAboutToBeUnloaded(QString)', self._onModuleAboutToBeUnloaded)
        self.browserWidget = qt.QWidget()
        self.browserWidget.setWindowTitle('TCIA Browser')
        
        
    def resourcePath(self, filename):
        scriptedModulesPath = os.path.dirname(slicer.util.modulePath(self.moduleName))
        return os.path.join(scriptedModulesPath, 'Resources', filename)

    def cleanup(self):
        """Override this function to implement module widget specific cleanup.

    #     It is invoked when the signal `qSlicerModuleManager::moduleAboutToBeUnloaded(QString)`
    #     corresponding to the current module is emitted and just before a module is
    #     effectively unloaded.
    #     """
    #     pass

    def _onModuleAboutToBeUnloaded(self, moduleName):
        """This slot calls `cleanup()` if the module about to be unloaded is the
        current one.
        """
        if moduleName == self.moduleName:
            self.cleanup()
            slicer.app.moduleManager().disconnect(
                'moduleAboutToBeUnloaded(QString)', self._onModuleAboutToBeUnloaded)     
    
    def setupDeveloperSection(self):
        if not self.developerMode:
            return

        self.reloadCollapsibleButton = ctk.ctkCollapsibleButton()
        self.reloadCollapsibleButton.text = "Reload && Test"
        self.layout.addWidget(self.reloadCollapsibleButton)
        reloadFormLayout = qt.QHBoxLayout(self.reloadCollapsibleButton)

        # reload button
        self.reloadButton = qt.QPushButton("Reload")
        self.reloadButton.toolTip = "Reload this module."
        self.reloadButton.name = "ScriptedLoadableModuleTemplate Reload"
        self.reloadButton.connect('clicked()', self.onReload)

        # reload and test button
        self.reloadAndTestButton = qt.QPushButton("Reload and Test")
        self.reloadAndTestButton.toolTip = "Reload this module and then run the self tests."
        self.reloadAndTestButton.connect('clicked()', self.onReloadAndTest)

        self.restartButton = qt.QPushButton("Restart Slicer")
        self.restartButton.toolTip = "Restart Slicer"
        self.restartButton.name = "ScriptedLoadableModuleTemplate Restart"
        self.restartButton.connect('clicked()', slicer.app.restart)
        elements = [self.reloadButton, self.reloadAndTestButton, self.restartButton]
        for element in elements:
                reloadFormLayout.addWidget(element)
    

    def setup(self):
        # Instantiate and connect default widgets ...
        self.setupDeveloperSection()

        self.Markup = MarkupClass(self.layout)
        self.Markup.setupMarkup()
        
    def onReload(self):
        print('\n' * 2)
        print('-' * 30)
        print('Reloading module: ' + self.moduleName)
        print('-' * 30)
        print('\n' * 2)
        slicer.mrmlScene.Clear()

class MarkupClass(fist_InterfaceWidget):
    def __init__(self, layout): 
        self.layout = layout
        self.fiducialNode = slicer.vtkMRMLMarkupsFiducialNode()
        slicer.mrmlScene.AddNode(self.fiducialNode)
        self.fiducialNode.SetName("fiducial out")
        self.fiducialNode.GetDisplayNode().SetSelectedColor(1,1,0)
        self.fiducialNode.AddControlPointWorld(50, 0, 0)
        self.fiducialNode.AddControlPointWorld(0, 50, 0)
        self.fiducialNode.AddControlPointWorld(0, 0, 50)
        self.fiducialNode.GetDisplayNode().SetVisibility(False)
        self.MarkupList = ["fiducial_1", "fiducial_2", "fiducial_3"]
        

    def onMarkupChanged(self, caller, event):
        markupsNode = caller
        id1 = self.mMarkupid.count
        sliceView = markupsNode.GetAttribute("Markups.MovingInSliceView")
        movingMarkupIndex = markupsNode.GetDisplayNode().GetActiveControlPoint()
        if movingMarkupIndex >= 0:
            pos = [0,0,0]
            markupsNode.GetNthControlPointPosition(movingMarkupIndex, pos)
        
            if movingMarkupIndex != self.mMarkupid.count:
                self.mMarkupid.setCurrentIndex(movingMarkupIndex)
                self.coordX.setValue(pos[0])
                self.coordY.setValue(pos[1])
                self.coordZ.setValue(pos[2]) 
                self.selectionchange('Line')
    
    def AddNewMarkup(self, caller, event):
        markupsNode = caller
        sliceView = markupsNode.GetAttribute("Markups.PointPositionDefinedEvent")
        movingMarkupIndex = markupsNode.GetDisplayNode().GetActiveControlPoint()
        addingMarkup = slicer.vtkMRMLMarkupsNode.PointPositionDefinedEvent.__bool__()
        #print(addingMarkup)
        if addingMarkup:
            newMarkup = f"fiducial_{movingMarkupIndex+1}"
            self.mMarkupid.addItem(newMarkup)
            self.FistPoint.addItem(newMarkup)
            self.SecondPoint.addItem(newMarkup)
            self.mMarkupid.setCurrentIndex(movingMarkupIndex)
            

    def ResponseSwitch(self):        
        if self.mSwitch.isChecked():  
            self.fiducialNode.GetDisplayNode().SetVisibility(self.mSwitch.isChecked())
        else:
            self.fiducialNode.GetDisplayNode().SetVisibility(self.mSwitch.isChecked())

        if self.IGTSwitch.isChecked():
            self.IGTNode.Start()
       
        else:
            self.IGTNode.Stop()
    
    def EditMarkup(self, action, id=0):
        if action == "Add":
            self.selectionNode = slicer.mrmlScene.GetNodeByID("vtkMRMLSelectionNodeSingleton")
            self.selectionNode.SetReferenceActivePlaceNodeClassName("vtkMRMLMarkupsFiducialNode")
            self.interactionNode = slicer.mrmlScene.GetNodeByID("vtkMRMLInteractionNodeSingleton")
            placeModePersistence = 1
            self.interactionNode.SetPlaceModePersistence(placeModePersistence)
            self.interactionNode.SetCurrentInteractionMode(1)
            self.fiducialNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointPositionDefinedEvent, self.AddNewMarkup)
            
            
        if action == "Delete":
            self.mMarkupid.removeItem(id)
            self.fiducialNode.RemoveNthControlPoint(id)
            self.selectionchange('Points')

        if action == "Apply":
            #self.interactionNode = slicer.mrmlScene.GetNodeByID("vtkMRMLInteractionNodeSingleton")
            self.interactionNode.SwitchToViewTransformMode()
            self.interactionNode.SetPlaceModePersistence(0)
            self.mMarkupid.setCurrentIndex(id)
            self.fiducialNode.SetNthControlPointPosition(id, self.coordX.value, self.coordY.value, self.coordZ.value)
  
    def selectionchange(self, type):
        if type == 'Points':
            pose = self.fiducialNode.GetNthControlPointPosition(self.mMarkupid.currentIndex)
            self.coordX.setValue(pose[0])
            self.coordY.setValue(pose[1])
            self.coordZ.setValue(pose[2])
            self.fiducialNode.AddObserver(slicer.vtkMRMLMarkupsNode.PointModifiedEvent, self.onMarkupChanged)
            #self.LineDictionary['Line1'].AddObserver(slicer.vtkMRMLMarkupsNode.PointModifiedEvent, self.onMarkupChanged)

        if type == 'Line':
            FistPointPose = self.fiducialNode.GetNthControlPointPosition(self.FistPoint.currentIndex)
            SecondPointPose  = self.fiducialNode.GetNthControlPointPosition(self.SecondPoint.currentIndex)
            FinalPose = [0,0,0]
            for i in range(len(FistPointPose)):
                FinalPose[i] = FistPointPose[i] - SecondPointPose[i]
            Final = np.power(FinalPose, 2)
            Distance = np.sqrt(Final[0]+Final[1]+Final[2])
            self.ShowDistance.setText(f"{np.round(Distance, 3)}")
   
    def setupMarkup(self):
    
    # TESTE 
        self.fiducialCBox = slicer.qMRMLNodeComboBox()
        # MUITO IMPORTANTEEEEEEEEE!!!!!!!
        self.fiducialCBox.nodeTypes = (("vtkMRMLMarkupsFiducialNode"), ("vtkMRMLMarkupsLineNode"))
        self.fiducialCBox.selectNodeUponCreation = False
        self.fiducialCBox.addEnabled = False
        self.fiducialCBox.removeEnabled = False
        self.fiducialCBox.noneEnabled = True
        self.fiducialCBox.setMRMLScene(slicer.mrmlScene)
        self.fiducialCBox.setToolTip("Select a fiducial list")

    # FONT
        self.font = qt.QFont()
        self.font.setPixelSize(15) 
        
    # IGT AREA
        self.IGTCollapsibleButton = ctk.ctkCollapsibleButton()
        self.IGTCollapsibleButton.text = "IGT connection"

    # NODE 
        self.IGTNode = slicer.vtkMRMLIGTLConnectorNode()
        self.IGTNode.SetName('IGT Connector')
        slicer.mrmlScene.AddNode(self.IGTNode)
        self.IGTNode.SetTypeServer(18944)
        self.IGTNode.SetTypeClient('localhost', 18944)

    # LABEL
        self.layout.addWidget(self.IGTCollapsibleButton)
        IGTLayout = qt.QFormLayout(self.IGTCollapsibleButton)
        self.IGTLabelSwitch = qt.QLabel("IGT connection")
        self.IGTLabelSwitch.setFont(self.font)
    
    # BUTTON
        self.IGTSwitch = PyToogle()
        IGTLayout.addRow(self.IGTLabelSwitch, self.IGTSwitch)
    
    # CONNECTION
        self.IGTSwitch.stateChanged.connect(self.ResponseSwitch)
        
    
    # MARKUP AREA
        self.MarkupsCollapsibleButton = ctk.ctkCollapsibleButton()
        self.MarkupsCollapsibleButton.text = "Fiducial point"
        self.layout.addWidget(self.MarkupsCollapsibleButton)
        #self.layout.addSpacing(20)                

    # BUTTON
        self.mAdd = qt.QPushButton("Add")
        self.mDelete = qt.QPushButton("Delete")
        self.mApply = qt.QPushButton("Apply")
        self.mSwitch = PyToogle()

        # BUTTONS LAYOUT
        AddDeleteButton = qt.QGridLayout()
        AddDeleteButton.addWidget(self.mAdd, 0, 0)
        AddDeleteButton.addWidget(self.mDelete, 0, 1)
        AddDeleteButton.addWidget(self.mApply, 0, 2)
        
    # SPIN BOX
        self.coordX = qt.QDoubleSpinBox()
        self.coordX.setMaximum(2000)
        self.coordX.setDecimals(3)
        self.coordX.setMinimum(-2000) 

        self.coordY = qt.QDoubleSpinBox()
        self.coordY.setMaximum(2000) 
        self.coordY.setMinimum(-2000) 
        self.coordY.setDecimals(3)
        
        self.coordZ = qt.QDoubleSpinBox()
        self.coordZ.setMaximum(2000)
        self.coordZ.setMinimum(-2000)
        self.coordZ.setDecimals(3) 

    # LINE SECTION 
        # NODE
        self.FistPoint = qt.QComboBox()
        self.SecondPoint = qt.QComboBox()
        self.FistPoint.addItems(self.MarkupList)
        self.SecondPoint.addItems(self.MarkupList)
        self.FistPoint.currentIndexChanged.connect(lambda: self.selectionchange('Line'))
        self.SecondPoint.currentIndexChanged.connect(lambda: self.selectionchange('Line'))
        
        # TEXTBOX
        self.ShowDistance = qt.QLineEdit()

        # LABEL
        self.LabelLine = qt.QLabel("Distance Point")
        self.LabelLineInstructions = qt.QLabel('Select Points')
        self.LabelLine.setFont(self.font)
        self.Distance = qt.QLabel("Line Distance")

        # LAYOUT
        self.LineLayout = qt.QGridLayout()
        self.LineLayout.addWidget(self.LabelLineInstructions, 0, 0)
        self.LineLayout.addWidget(self.FistPoint, 0, 1)
        self.LineLayout.addWidget(self.SecondPoint, 0, 2)
        self.LineLayout.addWidget(self.Distance, 1, 0)
        self.LineLayout.addWidget(self.ShowDistance, 1, 1, 1, 2)
        
        
     
      
    # COMBO BOX
        self.mMarkupid = qt.QComboBox()
        self.mMarkupid.addItems(self.MarkupList)
        self.mMarkupid.currentIndexChanged.connect(lambda: self.selectionchange('Points'))
        
    # LABEL
        self.mLabelSwitch = qt.QLabel("Activate Markups")
        self.mLabelSwitch.setFont(self.font)
        self.mLabelEdit = qt.QLabel("Edition Markups")
        self.mid = qt.QLabel("Markup")
        self.mLabelEdit.setFont(self.font)
        self.coordXLabel = qt.QLabel("X Coordenate")
        self.coordYLabel = qt.QLabel("Y Coordenate")
        self.coordZLabel = qt.QLabel("Z Coordenate")
        

    # LAYOUT
        MarkupLayout = qt.QFormLayout(self.MarkupsCollapsibleButton)
        MarkupLayout.addRow(self.mLabelSwitch, self.mSwitch)
        MarkupLayout.addRow(self.mLabelEdit)
        MarkupLayout.addRow(self.mid, self.mMarkupid)
        MarkupLayout.addRow(self.coordXLabel, self.coordX)
        MarkupLayout.addRow(self.coordYLabel, self.coordY)
        MarkupLayout.addRow(self.coordZLabel, self.coordZ)
        MarkupLayout.addRow(AddDeleteButton)
        MarkupLayout.addRow(self.LabelLine)
        MarkupLayout.addRow(self.LineLayout)
        MarkupLayout.addRow(self.fiducialCBox)
        MarkupLayout.setVerticalSpacing(10)
        MarkupLayout.setFormAlignment(qt.Qt.AlignCenter)
        
    # APPLY BUTTONS
        self.mSwitch.stateChanged.connect(self.ResponseSwitch)
        self.mAdd.connect('clicked()',lambda:  self.EditMarkup("Add"))
        self.mDelete.connect('clicked()',lambda:  self.EditMarkup("Delete", self.mMarkupid.currentIndex))
        self.mApply.connect('clicked()',lambda: self.EditMarkup("Apply", self.mMarkupid.currentIndex))   
   
    # APPLY COMBO BOX
        self.selectionchange('Points')
        self.interactionNode = slicer.mrmlScene.GetNodeByID("vtkMRMLInteractionNodeSingleton")
        self.interactionNode.SetCurrentInteractionMode(0)
        
class PyToogle(qt.QCheckBox):
    def __init__(self, width = 40,  bg_color = '#777', circle_color = '#C4C4C4', active_color = '#0B8C05'):
        qt.QCheckBox.__init__(self)

        # Set default parameters
        self.setFixedSize(width, 22)
        self.setCursor(qt.Qt.PointingHandCursor)
        
        # Colors 
        self._bg_color = bg_color
        self._circle_color = circle_color
        self._active_color = active_color        

    # SET NEW HIT AREA
    def hitButton(self, pos: qt.QPoint):
        # Torna toda a ??rea do ret??ngulo reponsiva ao click
        return self.contentsRect().contains(pos)
        
    #DRAW NEW ITENS
    # Essa fun????o se sobrep??e no layout padr??o
    def paintEvent(self, e):
        
        # SET PAINTER
        painter = qt.QPainter(self)
        painter.setRenderHint(qt.QPainter.Antialiasing)

        # SET AS NO PEN
        painter.setPen(qt.Qt.NoPen)

        # DRAW RECTANGLE
        rect = qt.QRect(0, 0, self.width, self.height)

        #CHECK IF IS CHECKED
        if not self.isChecked():
            # DRAW BG
            painter.setBrush(qt.QColor(self._bg_color))
            painter.drawRoundedRect(0,0, rect.width(), self.height, self.height/2, 11)

            # DRAW CIRCLE 
            painter.setBrush(qt.QColor(self._circle_color))
            painter.drawEllipse(2, 2, 18, 18)

        else:
            # DRAW BG
            painter.setBrush(qt.QColor(self._active_color))
            painter.drawRoundedRect(0, 0, rect.width(), self.height, self.height/2, 11)

            # DRAW CIRCLE 
            painter.setBrush(qt.QColor(self._circle_color))
            painter.drawEllipse(self.width - 20, 2, 18, 18)

        # END DRAW
        painter.end()
        
class fist_InterfaceLogic(ScriptedLoadableModuleLogic):
    """This class should implement all the actual
    computation done by your module.  The interface
    should be such that other python code can import
    this class and make use of the functionality without
    requiring an instance of the Widget.
    Uses ScriptedLoadableModuleLogic base class, available at:
    https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self):
        """
        Called when the logic class is instantiated. Can be used for initializing member variables.
        """
        ScriptedLoadableModuleLogic.__init__(self)

    def setDefaultParameters(self, parameterNode):
        """
        Initialize parameter node with default settings.
        """
        if not parameterNode.GetParameter("Threshold"):
            parameterNode.SetParameter("Threshold", "100.0")
        if not parameterNode.GetParameter("Invert"):
            parameterNode.SetParameter("Invert", "false")

    def process(self, inputVolume, outputVolume, imageThreshold, invert=False, showResult=True):
        """
        Run the processing algorithm.
        Can be used without GUI widget.
        :param inputVolume: volume to be thresholded
        :param outputVolume: thresholding result
        :param imageThreshold: values above/below this threshold will be set to 0
        :param invert: if True then values above the threshold will be set to 0, otherwise values below are set to 0
        :param showResult: show output volume in slice viewers
        """

        if not inputVolume or not outputVolume:
            raise ValueError("Input or output volume is invalid")

        import time
        startTime = time.time()
        logging.info('Processing started')

        # Compute the thresholded output volume using the "Threshold Scalar Volume" CLI module
        cliParams = {
            'InputVolume': inputVolume.GetID(),
            'OutputVolume': outputVolume.GetID(),
            'ThresholdValue': imageThreshold,
            'ThresholdType': 'Above' if invert else 'Below'
        }
        cliNode = slicer.cli.run(slicer.modules.thresholdscalarvolume, None, cliParams, wait_for_completion=True, update_display=showResult)
        # We don't need the CLI module node anymore, remove it to not clutter the scene with it
        slicer.mrmlScene.RemoveNode(cliNode)

        stopTime = time.time()
        logging.info(f'Processing completed in {stopTime-startTime:.2f} seconds')


#
# fist_InterfaceTest
#

class fist_InterfaceTest(ScriptedLoadableModuleTest):
    """
    This is the test case for your scripted module.
    Uses ScriptedLoadableModuleTest base class, available at:
    https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def setUp(self):
        """ Do whatever is needed to reset the state - typically a scene clear will be enough.
        """
        slicer.mrmlScene.Clear()

    def runTest(self):
        """Run as few or as many tests as needed here.
        """
        self.setUp()
        self.test_fist_Interface1()

    def test_fist_Interface1(self):
        """ Ideally you should have several levels of tests.  At the lowest level
        tests should exercise the functionality of the logic with different inputs
        (both valid and invalid).  At higher levels your tests should emulate the
        way the user would interact with your code and confirm that it still works
        the way you intended.
        One of the most important features of the tests is that it should alert other
        developers when their changes will have an impact on the behavior of your
        module.  For example, if a developer removes a feature that you depend on,
        your test should break so they know that the feature is needed.
        """

        self.delayDisplay("Starting the test")

        # Get/create input data

        import SampleData
        registerSampleData()
        inputVolume = SampleData.downloadSample('fist_Interface1')
        self.delayDisplay('Loaded test data set')

        inputScalarRange = inputVolume.GetImageData().GetScalarRange()
        self.assertEqual(inputScalarRange[0], 0)
        self.assertEqual(inputScalarRange[1], 695)

        outputVolume = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLScalarVolumeNode")
        threshold = 100

        # Test the module logic

        logic = fist_InterfaceLogic()

        # Test algorithm with non-inverted threshold
        logic.process(inputVolume, outputVolume, threshold, True)
        outputScalarRange = outputVolume.GetImageData().GetScalarRange()
        self.assertEqual(outputScalarRange[0], inputScalarRange[0])
        self.assertEqual(outputScalarRange[1], threshold)

        # Test algorithm with inverted threshold
        logic.process(inputVolume, outputVolume, threshold, False)
        outputScalarRange = outputVolume.GetImageData().GetScalarRange()
        self.assertEqual(outputScalarRange[0], inputScalarRange[0])
        self.assertEqual(outputScalarRange[1], inputScalarRange[1])

        self.delayDisplay('Test passed')

