import qt
import os

class Button(qt.QPushButton):
    def __init__(self, name = '', width = 30):
        qt.QPushButton.__init__(self)

        # Set default parameters
        #self.setFixedSize(width, width)
        self.setCursor(qt.Qt.PointingHandCursor)
        self.name = name
        #self.width = width

        # get path of this file
        script_path = os.path.dirname(__file__)
        print("Script path ="+ script_path)

        # Set images path
        self.images = {
            "Add": "ImageButton/plus.png",
            "Delete": "ImageButton/delete.png",
            "Apply": "ImageButton/apply.png",
            "Rename": "ImageButton/rename.png",
            "Share": "ImageButton/share.png"
        } 

        # set images above with relative path
        self.images = {k: os.path.join(script_path, v) for k, v in self.images.items()}

        self.setImage(width-5)

    def setImage(self, size):
        self.setIcon(qt.QIcon(qt.QPixmap(self.images[self.name])))
        self.setIconSize(qt.QSize(size,size))
        self.setStyleSheet('text-align: center;')