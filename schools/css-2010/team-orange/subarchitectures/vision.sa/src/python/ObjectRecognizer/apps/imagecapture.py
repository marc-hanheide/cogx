#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: may 2009 
import os, sys, time, itertools
import math
import opencv.cv as cv
import opencv.highgui as hg
import opencv.adaptors as cvada

import pymodulepaths
import siftgpu
from ObjectRecognizer.mods.capture import CCameraCapture, copyFrame
from ObjectRecognizer.mods.numutil import *
import ObjectRecognizer.mods.canvas.cvcanvas as canvas
import ObjectRecognizer.mods.cameraview as camview
from ObjectRecognizer.objectmodel import CModelFileManager

class TextWriter:
    textHelp = [
        "Model Maker",
        "S   - Save viewpoint data",
        "D   - Change display mode",
        "N   - Next viewpoint",
        "+/- - Elevation",
        "X   - SIFT On/Off",
        "H   - Help On/Off",
        "Esc - Quit"
    ]
    textMiniHelp = [ "H - Help" ]
    MOD_MINI = 0
    MOD_FULL = 1

    def __init__(self, pixSize=9):
        line_type = 0 # cv.CV_AA
        self.coText = cv.cvScalar(32, 255, 32)
        font = cv.cvInitFont (cv.CV_FONT_HERSHEY_COMPLEX, 1, 1, 0.0, 1, line_type)
        text_size, ymin = cv.cvGetTextSize ("OpenCV", font)
        fac = pixSize * 1.0 / text_size.height
        self.font = cv.cvInitFont (cv.CV_FONT_HERSHEY_COMPLEX, fac, fac, 0.0, 1, line_type)
        text_size, ymin = cv.cvGetTextSize ("OpenCV", self.font)
        self.hline = text_size.height + ymin
        self.displayMode = TextWriter.MOD_FULL
        self.status = []

    def nextDisplayMode(self):
        if self.displayMode == TextWriter.MOD_FULL: self.displayMode = TextWriter.MOD_MINI
        else: self.displayMode = TextWriter.MOD_FULL

    def printStatus(self, img):
        orig = cv.cvPoint (5, self.hline)
        if self.displayMode == TextWriter.MOD_MINI: it = itertools.chain (TextWriter.textMiniHelp, self.status)
        else: it = itertools.chain (TextWriter.textHelp, self.status)
        for l in it:
            cv.cvPutText(img, l, orig, self.font, self.coText)
            orig.y += self.hline

    def setStatus(self, lineno, text):
        if text == None: text = ""
        while len(self.status) <= lineno: self.status.append("")
        self.status[lineno] = text

class CFakeModel:
    def __init__(self):
        self.name = None

class FeatureCapture:
    def __init__(self):
        self.elevations = [0, 15, 30, 45, 60, 75, 90]
        self.iElevation = 0
        self.viewpoint = 0
        self.TW = TextWriter()
        self.processing = False
        #self.SIFT = None
        #self.MATCH = None
        self.smoothImage = False
        self.camwin = "Camera"
        self.canvas = canvas.CCanvas(1024, 768)
        self.featurePack = None
        self.drawMode = 0
        self.drawModeName=["Preview", "Last Saved", "Preview and Last Saved"]
        self.model = CFakeModel()
        self.FM = CModelFileManager(self.model, "xdata")

        self.capture = CameraCapture(device=0, size=(640,480))
        # self.capture = CLoopback1394Capture(device=2)
        # self.capture.setSize((640, 480))
        # capture.setFrameRate(15)
        # self.capture.setBrightness(0.6)

        self.currentFrame = None
        self.lastSavedFrame = None
        self.featurePackSaved = None
        self.curRunSize = 0

    def updateStatus(self):
        self.TW.setStatus(
            0, "Model '%s'.%d SIFT: %s " % (
                self.model.name, self.viewpoint, "On" if self.processing else "OFF",
            )
        )
        self.TW.setStatus(1, "Elevation: %d, View: %s, Smooth: %s" % (
            self.elevations[self.iElevation],
            self.drawModeName[self.drawMode],
            "MEDIAN 3" if self.smoothImage else "Off"
            )
        )
        ns = 0 if self.featurePackSaved == None else len(self.featurePackSaved)
        nc = 0 if self.featurePack == None else len(self.featurePack)
        self.TW.setStatus(2, "Best: %d, Saved: %d" % (nc, ns))

    def drawKeypoints(self, keypoints, scale=1.0, offset=(0,0)):
        for r in keypoints:
            x, y = int(r[0]*scale+offset[0]), int(r[1]*scale+offset[1])
            rad = r[2] # r.scale
            ornt = r[3] # r.orientation
            x1 = int(x + scale * rad * math.cos(ornt))
            y1 = int( y + scale * rad * math.sin(ornt) )
            self.canvas.drawLine(x, y, x1, y1, color="blue")
            self.canvas.drawPoint(x, y, shape=".", color="red")

    def processCvKey(self, k):
        if k == 'H' or k == 'h':
            self.TW.nextDisplayMode()
            return -1
        elif k in ['D', 'd']:
            self.drawMode = (self.drawMode + 1) % 3
        elif k in ['N', 'n']:
            self.viewpoint += 1
            self.processing = False
            self.lastSavedFrame = None
            self.featurePackSaved = None
            self.featurePack = None
        elif k in ['+']:
            self.iElevation = self.iElevation + 1
            if self.iElevation >= len(self.elevations): self.iElevation = 0
        elif k in ['-']:
            self.iElevation = self.iElevation - 1
            if self.iElevation < 0: self.iElevation = len(self.elevations) - 1
        #elif k in ['N', 'n']:
        #    self.smoothImage = not self.smoothImage
        elif k in ['S', 's']:
            if self.currentFrame != None:
                basename = "%s/%s_VP%03d_I%03d" % (
                    self.FM.imageDir, self.model.name, self.elevations[self.iElevation], self.viewpoint)
                self.lastSavedFrame = self.currentFrame
                self.featurePackSaved = self.featurePack
                hg.cvSaveImage(basename + ".png", self.lastSavedFrame)
        elif k == 'X' or k == 'x':
            self.processing = not self.processing
            if self.processing:
                # -pack - not working with nvidia Quadro FX 550 (driver v.180.11)
                #if self.SIFT == None: self.SIFT = SiftGPU(params="-s")
                #if self.MATCH == None: self.MATCH = SiftMatchGPU(create_context=False)
                self.curRunSize = 0
        return k

    def _acquireSift(self, frame, smooth=False):
        if smooth:
            copy = cv.cvCreateImage(cv.cvSize(frame.width, frame.height), frame.depth, frame.nChannels)
            cv.cvSmooth(frame, copy, cv.CV_MEDIAN, 3, 3)
            frame = copy
        im = np.array(cvada.Ipl2NumPy(frame))
        return siftgpu.extractFeatures(im)

    def run(self):
        hg.cvNamedWindow(self.camwin, hg.CV_WINDOW_AUTOSIZE)
        self.capture.start()
        self.currentFrame = self.capture.grabFrameCopy()
        STOPAFTER = 4
        scale = 1; savedOffset=(0,0)
        self.FM.checkModelDirs()

        while 1:
            k = hg.cvWaitKey(10)
            k = self.processCvKey(k)
            if k == '\x1b': break
            self.currentFrame = self.capture.grabFrameCopy()
            if self.currentFrame is None: continue
            self.updateStatus()

            if self.drawMode == 0: scale = 1.0; savedOffset=(0,0)
            elif self.drawMode == 1: scale = 1.0; savedOffset=(0,0)
            elif self.drawMode == 2: scale = 0.5; savedOffset=(0,int(768*scale))

            if self.drawMode in [1, 2]:
                if self.lastSavedFrame == None: self.canvas.clear(color="black")
                else: self.canvas.drawImage(self.lastSavedFrame, savedOffset[0], savedOffset[1], size=scale)
                if self.featurePackSaved != None:
                    self.drawKeypoints(self.featurePackSaved.keypoints, scale=scale, offset=savedOffset)
            if self.drawMode in [0, 2]:
                self.canvas.drawImage(self.currentFrame, 0, 0, size=scale)
            if not self.processing:
                self.TW.printStatus(self.canvas.dc)
                hg.cvShowImage(self.camwin, self.canvas.dc)
                continue

            keys, features = self._acquireSift(self.currentFrame, smooth=self.smoothImage)
            if self.featurePack == None or len(keys) > len(self.featurePack):
                self.featurePack = camview.CFeaturepack(keys, features)
            if self.drawMode in [0, 2]:
                self.drawKeypoints(keys, scale=scale, offset=(0,0))
            self.curRunSize += 1
            # if self.curRunSize >= STOPAFTER: self.processing = False

            self.updateStatus()
            self.TW.printStatus(self.canvas.dc)
            hg.cvShowImage(self.camwin, self.canvas.dc)

def main():
    App = FeatureCapture()
    App.model.name = sys.argv[1]
    App.run()

if __name__ == "__main__": main()
