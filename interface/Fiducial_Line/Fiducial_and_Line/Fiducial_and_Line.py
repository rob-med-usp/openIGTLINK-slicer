import logging
import os
import numpy as np
import slicer, ctk, vtk, qt
from slicer.ScriptedLoadableModule import *
import sys

#
# Fiducial_and_Line
#

class Fiducial_and_Line(ScriptedLoadableModule):
    """Uses ScriptedLoadableModule base class, available at:
    https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = "Fiducial_and_Line"  # TODO: make this more human readable by adding spaces
        self.parent.categories = ["Examples"]  # TODO: set categories (folders where the module shows up in the module selector)
        self.parent.dependencies = []  # TODO: add here list of module names that this module requires
        self.parent.contributors = ["John Doe (AnyWare Corp.)"]  # TODO: replace with "Firstname Lastname (Organization)"
        # TODO: update with short description of the module and a link to online module documentation
        self.parent.helpText = """
This is an example of scripted loadable module bundled in an extension.
See more information in <a href="https://github.com/organization/projectname#Fiducial_and_Line">module documentation</a>.
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

    # Fiducial_and_Line1
    SampleData.SampleDataLogic.registerCustomSampleDataSource(
        # Category and sample name displayed in Sample Data module
        category='Fiducial_and_Line',
        sampleName='Fiducial_and_Line1',
        # Thumbnail should have size of approximately 260x280 pixels and stored in Resources/Icons folder.
        # It can be created by Screen Capture module, "Capture all views" option enabled, "Number of images" set to "Single".
        thumbnailFileName=os.path.join(iconsPath, 'Fiducial_and_Line1.png'),
        # Download URL and target file name
        uris="https://github.com/Slicer/SlicerTestingData/releases/download/SHA256/998cb522173839c78657f4bc0ea907cea09fd04e44601f17c82ea27927937b95",
        fileNames='Fiducial_and_Line1.nrrd',
        # Checksum to ensure file integrity. Can be computed by this command:
        #  import hashlib; print(hashlib.sha256(open(filename, "rb").read()).hexdigest())
        checksums='SHA256:998cb522173839c78657f4bc0ea907cea09fd04e44601f17c82ea27927937b95',
        # This node name will be used when the data set is loaded
        nodeNames='Fiducial_and_Line1'
    )

    # Fiducial_and_Line2
    SampleData.SampleDataLogic.registerCustomSampleDataSource(
        # Category and sample name displayed in Sample Data module
        category='Fiducial_and_Line',
        sampleName='Fiducial_and_Line2',
        thumbnailFileName=os.path.join(iconsPath, 'Fiducial_and_Line2.png'),
        # Download URL and target file name
        uris="https://github.com/Slicer/SlicerTestingData/releases/download/SHA256/1a64f3f422eb3d1c9b093d1a18da354b13bcf307907c66317e2463ee530b7a97",
        fileNames='Fiducial_and_Line2.nrrd',
        checksums='SHA256:1a64f3f422eb3d1c9b093d1a18da354b13bcf307907c66317e2463ee530b7a97',
        # This node name will be used when the data set is loaded
        nodeNames='Fiducial_and_Line2'
    )


#
# Fiducial_and_LineWidget
#

class Fiducial_and_LineWidget(ScriptedLoadableModuleWidget):
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

        def createHLayout(elements):
            rowLayout = qt.QHBoxLayout()
            for element in elements:
                rowLayout.addWidget(element)
            return rowLayout

        #
        # Reload and Test area
        # Used during development, but hidden when delivering
        # developer mode is turned off.

        self.reloadCollapsibleButton = ctk.ctkCollapsibleButton()
        self.reloadCollapsibleButton.text = "Reload && Test"
        self.layout.addWidget(self.reloadCollapsibleButton)
        reloadFormLayout = qt.QFormLayout(self.reloadCollapsibleButton)

        # reload button
        self.reloadButton = qt.QPushButton("Reload")
        self.reloadButton.toolTip = "Reload this module."
        self.reloadButton.name = "ScriptedLoadableModuleTemplate Reload"
        self.reloadButton.connect('clicked()', self.onReload)

        # reload and test button
        self.reloadAndTestButton = qt.QPushButton("Reload and Test")
        self.reloadAndTestButton.toolTip = "Reload this module and then run the self tests."
        self.reloadAndTestButton.connect('clicked()', self.onReloadAndTest)

        # edit python source code
        self.editSourceButton = qt.QPushButton("Edit")
        self.editSourceButton.toolTip = "Edit the module's source code."
        self.editSourceButton.connect('clicked()', self.onEditSource)

        self.editModuleUiButton = None
        moduleUiFileName = self.resourcePath('UI/%s.ui' % self.moduleName)
        import os.path
        if os.path.isfile(moduleUiFileName):
            # Module UI file exists
            self.editModuleUiButton = qt.QPushButton("Edit UI")
            self.editModuleUiButton.toolTip = "Edit the module's .ui file."
            self.editModuleUiButton.connect('clicked()', lambda filename=moduleUiFileName: slicer.util.startQtDesigner(moduleUiFileName))

        # restart Slicer button
        # (use this during development, but remove it when delivering
        #  your module to users)
        self.restartButton = qt.QPushButton("Restart Slicer")
        self.restartButton.toolTip = "Restart Slicer"
        self.restartButton.name = "ScriptedLoadableModuleTemplate Restart"
        self.restartButton.connect('clicked()', slicer.app.restart)

        if self.editModuleUiButton:
            # There are many buttons, distribute them in two rows
            reloadFormLayout.addRow(createHLayout([self.reloadButton, self.reloadAndTestButton, self.restartButton]))
            reloadFormLayout.addRow(createHLayout([self.editSourceButton, self.editModuleUiButton]))
        else:
            reloadFormLayout.addRow(createHLayout([self.reloadButton, self.reloadAndTestButton, self.editSourceButton, self.restartButton]))

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
        slicer.util.reloadScriptedModule(self.moduleName)
        slicer.mrmlScene.Clear()

class MarkupClass(Fiducial_and_LineWidget):
    def __init__(self, layout): 
        self.layout = layout
        self.Lines = []
        self.NameLine = []
        self.confirm = False

    def ResponseSwitch(self):        
        if self.IGTSwitch.isChecked():
            self.IGTNode.Start()
        
        else:
            self.IGTNode.Stop()
                
    def selectionchange(self):
        if len(self.NameLine)>0:
            select = slicer.mrmlScene.GetNodeByID("vtkMRMLSelectionNodeSingleton")
            self.id = self.SelectionNode.currentIndex
            select.SetActivePlaceNodeID(slicer.util.getNode(self.NameLine[self.id]).GetID())
            if self.confirm:
                self.node.Observer()
            self.ResponseSwitchLine()
            self.RenameResponse()
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
        idx = len(self.Lines)
        self.confirm = True

        # if self.LineName.text != 'Line Name' and self.LineName.text != '':
        #     name = self.LineName.text
        #     self.Lines.append(LineNode(name, idx))
        #     self.NameLine.append(name)
            
        
        # else:
        #     self.Lines.append(LineNode(f'Line{idx+1}', idx))
        #     self.NameLine.append(f'Line{idx+1}')

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
        
############################################################################################        
   
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
        self.IGTLabelSwitch.setFont(font)
    
    # BUTTON
        self.IGTSwitch = PyToogle()
        IGTLayout.addRow(self.IGTLabelSwitch, self.IGTSwitch)
    
    # CONNECTION
        self.IGTSwitch.stateChanged.connect(self.ResponseSwitch)

############################################################################################      
    # LINE AREA

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
        
class Button(qt.QPushButton):
    def __init__(self, name = '', width = 30):
        qt.QPushButton.__init__(self)

        # Set default parameters
        #self.setFixedSize(width, width)
        self.setCursor(qt.Qt.PointingHandCursor)
        self.name = name
        #self.width = width

        self.images = {
            "Add": "/home/eduardo/openIGTLINK-slicer/interface/Fiducial_Line/Fiducial_and_Line/ImageButton/plus.png",
            "Delete": "/home/eduardo/openIGTLINK-slicer/interface/Fiducial_Line/Fiducial_and_Line/ImageButton/delete.png",
            "Apply": "/home/eduardo/openIGTLINK-slicer/interface/Fiducial_Line/Fiducial_and_Line/ImageButton/apply.png",
            "Rename": "/home/eduardo/openIGTLINK-slicer/interface/Fiducial_Line/Fiducial_and_Line/ImageButton/rename.png",
        } 
        self.setImage(width-5)

    def setImage(self, size):
        self.setIcon(qt.QIcon(qt.QPixmap(self.images[self.name])))
        self.setIconSize(qt.QSize(size,size))
        self.setStyleSheet('text-align: center;')

        
        
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
        # Torna toda a área do retângulo reponsiva ao click
        return self.contentsRect().contains(pos)
        
    #DRAW NEW ITENS
    # Essa função se sobrepõe no layout padrão
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
        
class Fiducial_and_LineLogic(ScriptedLoadableModuleLogic):
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
# Fiducial_and_LineTest
#

class Fiducial_and_LineTest(ScriptedLoadableModuleTest):
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
        self.test_Fiducial_and_Line1()

    def test_Fiducial_and_Line1(self):
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
        inputVolume = SampleData.downloadSample('Fiducial_and_Line1')
        self.delayDisplay('Loaded test data set')

        inputScalarRange = inputVolume.GetImageData().GetScalarRange()
        self.assertEqual(inputScalarRange[0], 0)
        self.assertEqual(inputScalarRange[1], 695)

        outputVolume = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLScalarVolumeNode")
        threshold = 100

        # Test the module logic

        logic = Fiducial_and_LineLogic()

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

