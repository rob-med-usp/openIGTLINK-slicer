import qt

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
            "Share": "/home/eduardo/openIGTLINK-slicer/interface/Fiducial_Line/Fiducial_and_Line/ImageButton/share.png"
        } 
        self.setImage(width-5)

    def setImage(self, size):
        self.setIcon(qt.QIcon(qt.QPixmap(self.images[self.name])))
        self.setIconSize(qt.QSize(size,size))
        self.setStyleSheet('text-align: center;')