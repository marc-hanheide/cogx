#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim

from PyQt4 import QtCore, QtGui
from PyQt4.QtCore import pyqtSignature, pyqtProperty
import math

class CQwCameraPlacement(QtGui.QWidget):
    def __init__(self, parent = None):
        QtGui.QWidget.__init__(self, parent)
        self.camPhi = 0
        self.camLambda = 0

    # Longitude-----------
    def longitude(self):
        return self.camLambda

    @pyqtSignature("QString")
    @pyqtSignature("double")
    @pyqtSignature("int")
    def setLongitude(self, longitude):
        try: longitude = int(longitude)
        except: longitude = 0
        if longitude != self.camLambda:
            self.camLambda = longitude
            self.update()
    longitude = pyqtProperty("double", longitude, setLongitude)

    # Latitude----------- 
    def latitude(self):
        return self.camPhi

    @pyqtSignature("QString")
    @pyqtSignature("double")
    @pyqtSignature("int")
    def setLatitude(self, latitude):
        try: latitude = int(latitude)
        except: latitude = 0
        if latitude != self.camPhi:
            self.camPhi = latitude
            self.update()
    latitude = pyqtProperty("double", latitude, setLatitude)

    def paintEvent(self, event):
        brTable = QtGui.QBrush(QtGui.QColor("brown"))
        brCamera = QtGui.QBrush(QtGui.QColor("black"))
        coObject = QtGui.QColor("blue")
        dc = QtGui.QPainter()
        dc.begin(self)
        dc.setRenderHint(QtGui.QPainter.Antialiasing, True)
        def rectObject(w, h, t, l):
            dc.setPen(coObject)
            x = -w/2; y = -h/2
            off = int(math.sqrt(w*w/4 + h*h/4))
            dc.translate(l+off, t+off)
            dc.rotate(self.camLambda)
            dc.drawLine(x, y+h, x, y)
            dc.drawLine(x, y, x+w, y)
            dc.drawLine(x+w, y, x+w, y+h)
            dc.setPen(QtGui.QColor("red"))
            dc.drawLine(x+w, y+h, x, y+h)
            dc.setWorldTransform(QtGui.QTransform()) # reset
            return
        def camera(x, y, angle):
            dc.translate(x, y)
            dc.rotate(-angle)
            dc.setPen(coObject)
            dc.drawLine(0, 0, 15, -7)
            dc.drawLine(0, 0, 15, 7)
            dc.fillRect(-10, -5, 15, 10, brCamera)
            dc.setWorldTransform(QtGui.QTransform()) # reset
            return
        rectObject(55, 34, 5, 5)
        camera(38, 90, 90)
        dcam = 40; cx = 140; cy = 60
        dc.fillRect(cx-15, cy+10, 30, 5, brTable)
        dc.fillRect(cx-10, cy+15, 3, 10, brTable)
        dc.fillRect(cx+ 7, cy+15, 3, 10, brTable)
        dc.setPen(coObject)
        dc.drawRect(cx-5, cy-10, 10, 20)
        an = self.camPhi * math.pi
        dx = math.cos(an / 180) * dcam; dy = math.sin(an / 180) * dcam
        camera (cx + dx, cy - dy, self.camPhi + 180)

        dc.setFont(QtGui.QFont('Decorative', 10))
        dc.drawText(10, 120, "Labmda")
        dc.drawText(cx-10, 120, "Phi")
        dc.end()

