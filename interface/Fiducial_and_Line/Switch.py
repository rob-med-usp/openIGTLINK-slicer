import qt

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
