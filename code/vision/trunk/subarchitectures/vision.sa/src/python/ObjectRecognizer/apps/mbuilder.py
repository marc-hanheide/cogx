#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: Aug 2009 

import os, sys, time
import os.path
from PyQt4 import QtCore, QtGui

import opencv.cv as cv
import opencv.highgui as hg
import opencv.adaptors as cvada
import ctypes

import pymodulepaths
# import siftgpu
from ObjectRecognizer.mods.capture import CCameraCapture, copyFrame
from ObjectRecognizer.mods.numutil import *
import ObjectRecognizer.mods.cameraview as camview
import ObjectRecognizer.mods.capture as capture
import ObjectRecognizer.objectmodel as model
import ObjectRecognizer.objectmatcher as matcher
import ObjectRecognizer.mods.comparator as comparator
from ObjectRecognizer.qtmods import cameraChooser, qtimage
from ObjectRecognizer import featuresetup

from dlgCaptureSetup import CCaptureSetupDlg, CCaptureParams
import qtui.uiModelBuilder as uiModelBuilder

class CImageInfo:
    def __init__(self, title = None, preview = None):
        self.vpPhi = 0
        self.vpLambda = 0
        self._preview = preview
        self._title = title

    def title(self):
        return self._title

    def preview(self):
        return self._preview

class CModelBuilder:
    def __init__(self, model, extractor):
        self.model = model
        self.extractor = extractor

    # Create viewpoint from image and extract features
    def _createViewpoint(self, fnImage):
        vp = camview.CViewPoint()
        # image = hg.cvLoadImage(fnImage) # TODO: qtimage -> cvmat
        qim = QtGui.QImage()
        qim.load(fnImage)
        vp.setImage(image)
        npimg = cvada.Ipl2NumPy(image)
        fpack = extractor.extractFeatures(npimg)
        vp.featurePacks.append(fpack)
        phil = self.model.FM.parsePhiLambda(os.path.basename(fnImage))
        if phil != None:
            vp.vpPhi = phil[0]
            vp.vpLambda = phil[1]
        return vp

    def _imagesToFeatures(self):
        fm = self.model.FM
        try: imagefiles = sorted(os.listdir(fm.imageDir))
        except: imagefiles = []
        imagefiles = [im[:-4] for im in imagefiles if im.endswith(".png") ]
        try: featurefiles = sorted(os.listdir(fm.featureDir))
        except: featurefiles = []
        featurefiles = [fe[:-12] for fe in featurefiles if fe.endswith(".view.dat.gz") ]

        def featureFile(im): return "%s/%s.view.dat.gz" % (fm.featureDir, im)
        def imageFile(im): return "%s/%s.png" % (fm.imageDir, im)

        # WARNINIG: these are o(n^2); could be improved
        updimg = [im for im in imagefiles if im in featurefiles]
        newimg = [im for im in imagefiles if im not in featurefiles]
        delimg = [im for im in featurefiles if im not in imagefiles]

        # check timestamps for maybe-updated images
        toUpdate = []
        for im in updimg:
            stim = os.stat(imageFile(im))
            stft = os.stat(featureFile(im))
            if stim.st_mtime > stft.st_mtime: toUpdate.append(im)
        toUpdate.extend(newimg)

        for im in delimg:
            print "Deleting feature file '%s'" % im
            try: os.remove(featureFile(im))
            except:
                print " *** Failed"

        if len(toUpdate) > 0:
            if not os.path.exists(fm.featureDir): os.makedirs(fm.featureDir)

        for im in toUpdate:
            print "Creating feature file '%s'" % im
            vp = self._createViewpoint(imageFile(im))
            vp.save(featureFile(im))
            # print " *** Failed"

    def build(self):
        self._imagesToFeatures()


class CImageListModel(QtCore.QAbstractListModel):
    def __init__(self, parent = None):
        QtCore.QAbstractItemModel.__init__(self, parent)
        self.imageList = []

    def setList(self, imageList):
        self.beginRemoveRows(QtCore.QModelIndex(), 0, len(self.imageList))
        self.imageList = []
        self.endRemoveRows()
        self.imageList = imageList

    def parent(self, index):
        return QtCore.QModelIndex()

    def rowCount(self, parent=QtCore.QModelIndex()):
        return len(self.imageList)

    def columnCount(self, parent=QtCore.QModelIndex()):
        return 1
    
    def data(self, index, role=QtCore.Qt.DisplayRole):
        if not index.isValid(): return QtCore.QVariant()
        if index.row() < 0 or index.row() >= len(self.imageList):
            return QtCore.QVariant()

        if role == QtCore.Qt.DisplayRole:
            return QtCore.QVariant(self.imageList[index.row()].title())
        elif role == QtCore.Qt.DecorationRole:
            return QtCore.QVariant(self.imageList[index.row()].preview())

        return QtCore.QVariant()

class CModelBuilderWnd(QtGui.QMainWindow):
    def __init__(self):
        QtGui.QMainWindow.__init__(self)
        self.ui = uiModelBuilder.Ui_ModelBuilder()
        self.ui.setupUi(self)
        self.rawTitle = self.windowTitle()

        # Prepare camera view
        self.ui.scrollArea.setWidgetResizable(False)
        self.canvasScale = 1.0
        self.canvas = QtGui.QPixmap(640, 480)
        self.canvas.fill()
        self.ui.cameraView.setScaledContents(True)
        self.ui.cameraView.setPixmap(self.canvas)
        self.adjustCameraViewSize()
        self.resize(800, 600)

        # Prepare data-models for the widgets
        self.ui.cbElevation.clear()
        self.ui.cbElevation.addItem("0")
        self._imageList = CImageListModel()
        self.ui.lsvImages.setModel(self._imageList)

        # Prepare camera timer. TODO: Should use a separate thread for camera display
        self.devCapture = None
        self.tmCamera = QtCore.QTimer()
        self.connect(self.tmCamera, QtCore.SIGNAL("timeout()"), self.onUpdateCamera)

        # Data for the object model
        self.modelDir = os.getcwd()
        self.modelName = None
        self.model = None
        
        # Parameters for naming views
        self.paramCapture = CCaptureParams()

        # Camera input processing
        self.siftSetup = None
        self.frame = None
        self.siftPoints = None # CFeaturepack

        # Setup actions and events
        self.connect(self.ui.actionQuit, QtCore.SIGNAL("triggered()"), self.close)
        self.connect(self.ui.actionOpen, QtCore.SIGNAL("triggered()"), self.onOpenModel)
        self.connect(self.ui.actionSelectCamera, QtCore.SIGNAL("triggered()"), self.onSelectCamera)
        self.connect(self.ui.actionStartCamera, QtCore.SIGNAL("triggered()"), self.onStartCamera)
        self.connect(self.ui.actionStopCamera, QtCore.SIGNAL("triggered()"), self.onStopCamera)
        self.connect(self.ui.actionLoadElevationPreviews, QtCore.SIGNAL("triggered()"), self.onLoadPreviews)
        self.connect(self.ui.actCaptureSetup, QtCore.SIGNAL("triggered()"), self.onSetupCapture)
        self.connect(self.ui.actNextLambda, QtCore.SIGNAL("triggered()"), self.onNextLambda)
        self.connect(self.ui.actSaveView, QtCore.SIGNAL("triggered()"), self.onSaveView)
        self.connect(self.ui.actBuildModel, QtCore.SIGNAL("triggered()"), self.onBuildModel)

    def zoom(self, factor):
        if self.canvas == None: return
        nf = self.canvasScale * factor
        ns = self.canvas.size() * nf
        if ns.width() < 60: return
        if ns.width() > 2000: return
        self.canvasScale = nf
        self.adjustCameraViewSize()

    def adjustCameraViewSize(self):
        if self.canvas == None: return
        self.ui.cameraView.resize(self.canvas.size() * self.canvasScale)
        self.ui.scrollAreaWidgetContents.adjustSize()

    def isSiftEnabled(self):
        return self.ui.ckCalcSift.isChecked()

    def drawSiftPoints(self, painter):
        dc = painter
        dc.setPen(QtGui.QColor("magenta"))
        for p in self.siftPoints.keypoints:
            dc.drawLine(p[0], p[1], p[0]+2, p[1])

    def onUpdateCamera(self):
        if self.devCapture == None: return
        if not self.devCapture.isRunning(): return
        self.frame = self.devCapture.grabFrame(copy=True)
        if self.isSiftEnabled() and self.siftSetup != None:
            cvimg = cvada.Ipl2NumPy(self.frame)
            self.siftPoints = self.siftSetup.extractor.extractFeatures(cvimg)

        qimg = qtimage.qtImageFromIpl(self.frame)
        dc = QtGui.QPainter(self.canvas)
        dc.drawImage(0, 0, qimg)
        try:
            if self.isSiftEnabled() and self.siftPoints != None: self.drawSiftPoints(dc)
        except: pass
        dc.end()
        self.ui.cameraView.setPixmap(self.canvas)
        self.adjustCameraViewSize()
        # print time.time()

    def on_ckCalcSift_toggled(self, checked):
        if checked and self.siftSetup == None:
            cs = featuresetup.CSiftSetup
            self.siftSetup = cs(extractor=cs.GPU, matcher=cs.NUMPY)

    def on_btZoomIn_clicked(self, valid=True):
        if not valid: return
        self.zoom(1.25)

    def on_btZoomOut_clicked(self, valid=True):
        if not valid: return
        self.zoom(0.8)

    def onSelectCamera(self):
        self.tmCamera.stop()
        if self.devCapture: self.devCapture.stop()
        dlg = cameraChooser.CCameraChooserDlg()
        rv = dlg.exec_()
        if rv: self.prepareCapture(dlg.selected)

    def prepareCapture(self, cameraInfo):
        self.devCapture = capture.CCameraCapture(device=cameraInfo.id)
        self.devCapture.start()
        self.tmCamera.start(1000.0 / self.devCapture.fps)

    def onStartCamera(self):
        if self.devCapture == None: return
        if self.devCapture.isRunning(): self.devCapture.stop()
        self.devCapture.start()
        self.tmCamera.start(1000.0 / self.devCapture.fps)

    def onStopCamera(self):
        self.tmCamera.stop()
        if self.devCapture != None: self.devCapture.stop()

    def onOpenModel(self):
        qfd = QtGui.QFileDialog
        mdir = qfd.getExistingDirectory(
            self, "Open directory with object model", self.modelDir,
            qfd.ShowDirsOnly | qfd.DontResolveSymlinks
        )
        if mdir != None and len(mdir) > 1:
            self.loadModel("%s" % mdir)

    def loadModel(self, modelPath):
        self.modelDir, self.modelName = os.path.split(modelPath)
        self.setWindowTitle("%s - %s" % (self.modelName, self.rawTitle))
        self.model = model.CObjectModel(self.modelName)
        fm = self.model.FM
        fm.setModelStorePath(self.modelDir)

        # extract Phis
        try: images = os.listdir(fm.imageDir)
        except: images = []
        images = [i for i in images if i.endswith(".png")]
        philam = [fm.parsePhiLambda(img) for img in images]
        phis = [pl[0] for pl in philam if pl != None]
        phiidx = [fm.parsePhiIndex(img) for img in images]
        phis.extend([pl[0] for pl in phiidx if pl != None])
        phis.extend(self.paramCapture.phiList)
        phis = sorted(set(phis))
        if len(phis) < 1: phis = [0]

        # Phis to combo box
        self.ui.cbElevation.clear()
        for it in phis: self.ui.cbElevation.addItem("%d" % it)

    def getSelectedElevation(self):
        sel = "%s" % self.ui.cbElevation.currentText()
        try: sel = int(sel)
        except: sel = None
        return sel

    def getSelectedLambda(self):
        lmbd = "%s" % self.ui.txtLambda.text()
        try: lmbd = int(lmbd)
        except: lmbd = None
        return lmbd

    def onNextLambda(self):
        lmb = "%s" % self.ui.txtLambda.text()
        try: lmb = int(lmb.strip())
        except: lmb = 0
        step = -1 if self.paramCapture.lambdaReverse else 1
        lmb = (lmb + step * self.paramCapture.lambdaStep) % 360
        self.ui.txtLambda.setText("%d" % lmb)

    def loadPreviews(self, elevation = None):
        if self.model == None:
            self._imageList.setList([])
            return

        if elevation == None: sel = None
        else: sel = "VP%03d" % elevation

        fm = self.model.FM
        try: imagefiles = sorted(os.listdir(fm.imageDir))
        except: imagefiles = []
        images = []
        for fn in imagefiles:
            name, e = os.path.splitext(fn)
            name = name[len(self.modelName)+1:]
            if sel != None and not name.startswith(sel):
                continue # apply elevation filter
            im = QtGui.QImage()
            im.load(os.path.join(fm.imageDir, fn))
            pv = im.scaledToHeight(120)

            info = CImageInfo(name, pv)
            philam = fm.parsePhiLambda(fn)
            if philam != None:
                info.vpPhi = philam[0]
                info.vpLambda = philam[1]
            images.append(info)
        self._imageList.setList(images)

    # TODO: load images for selected phi
    def onLoadPreviews(self):
        QtGui.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        self.loadPreviews(self.getSelectedElevation())
        QtGui.QApplication.restoreOverrideCursor()

    def onSetupCapture(self):
        dlg = CCaptureSetupDlg()
        dlg.useParams(self.paramCapture)
        rv = dlg.exec_()

    def canSaveView(self):
        if not self.devCapture.isRunning(): return False
        if self.model == None: return False
        if self.getSelectedElevation() == None: return False
        if self.getSelectedLambda() == None: return False
        return True

    def onSaveView(self):
        if not self.canSaveView(): return
        fm = self.model.FM
        phi = self.getSelectedElevation()
        lmbda = self.getSelectedLambda()
        fname = "%s/%s.png" % (fm.imageDir, fm.viewpointBase2(phi, lmbda))
        name = "VP_%03d_L%03d" % (phi, lmbda)

        # hg.cvSaveImage(fname, frame) ceased working in new version of opencv (SVN 2034)...
        frame = self.devCapture.grabFrame(copy=True)
        qimg = qtimage.qtImageFromIpl(frame)
        if not os.path.exists(fm.imageDir): os.makedirs(fm.imageDir)
        qimg.save(fname)

        pv = qimg.scaledToHeight(120)

        info = None
        images = self._imageList.imageList
        for im in images:
            if im.title() == name:
                info = im
                break
        if info == None:
            info = CImageInfo(name, pv)
            images.append(info)
        else: info._preview = pv
        self._imageList.setList(images)
        
    def onBuildModel(self):
        mb = CModelBuilder(self.model, self.siftSetup)
        mb.build()

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    winmain = CModelBuilderWnd()
    winmain.show()
    sys.exit(app.exec_())

