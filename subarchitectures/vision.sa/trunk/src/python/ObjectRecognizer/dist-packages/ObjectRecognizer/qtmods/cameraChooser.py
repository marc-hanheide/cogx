#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim

import os, sys, time
import os.path
from PyQt4 import QtCore, QtGui
#import opencv.cv as cv
#import opencv.highgui as hg
#import opencv.adaptors as cvada
import cv
from qtui import uiSelectCamera
import qtimage

class CCameraInfo:
    # imgPreview: QImage or QPixmap
    def __init__(self, id, imgPreview, descr):
        self.id = id
        self.descr = descr
        self.preview = imgPreview
        # self.modes = [] # (w, h, fmt)

class CCameraList:
    def __init__(self):
        self.items = []
        pass

    def itemCount(self):
        return len(self.items)

    def refreshList(self): # OPENCV only
        # cv 1.9 subs = [cv.CV_CAP_V4L2, cv.CV_CAP_IEEE1394]
        subs = [ 0, 100 ]
        devnames = [("/dev/video%d",), ("/dev/video1394/%d", "/dev/video1394-%d")]
        subnames = ["V4L2", "IEEE1394"]
        nids = 4
        self.cameras = []
        for si, sub in enumerate(subs):
            for idev in xrange(nids):
                try:
                    camid = sub + idev
                    print camid
                    hasdev = False
                    for dev in devnames[si]:
                        if os.path.exists(dev % idev): hasdev = True
                    cap = None
                    if hasdev:
                        cap = cv.CaptureFromCAM(camid)
                        # can't set properties for firewire :(
                        #hg.cvSetCaptureProperty(cap, hg.CV_CAP_PROP_FRAME_WIDTH, 320.0)
                        #hg.cvSetCaptureProperty(cap, hg.CV_CAP_PROP_FRAME_HEIGHT, 240.0)
                        #hg.cvSetCaptureProperty(cap, hg.CV_CAP_PROP_MODE, 75)
                        img = cv.QueryFrame(cap)
                        if img != None:
                            # hg.cvSaveImage("%s-%d.png" % (subnames[si], idev), img)
                            name = "%s-%d (cvid=%d) %dx%dx%d (%d bit)" % (
                                subnames[si], idev, camid,
                                img.width, img.height, img.nChannels, img.depth
                                )
                            thumb = qtimage.qtImageThumbFromIpl(img, height=120)
                            self.items.append(CCameraInfo(camid, thumb, name))
                            print name
                        else:
                            print "No image from ", camid, dev % idev
                    #if cap != None: cv.ReleaseCapture(cap)
                except Exception as e:
                    print "Failed with ", camid, dev % idev
                    print e
                    pass

class CCameraListModel(QtCore.QAbstractListModel):
    def __init__(self, parent = None):
        QtCore.QAbstractItemModel.__init__(self, parent)
        self.cameraList = CCameraList()

    def refreshList(self):
        self.beginRemoveRows(QtCore.QModelIndex(), 0, self.cameraList.itemCount())
        self.cameraList.items = []
        self.endRemoveRows()
        self.cameraList.refreshList()

    def parent(self, index):
        return QtCore.QModelIndex()

    def rowCount(self, parent=QtCore.QModelIndex()):
        return self.cameraList.itemCount()

    def columnCount(self, parent=QtCore.QModelIndex()):
        return 1
    
    def data(self, index, role=QtCore.Qt.DisplayRole):
        if not index.isValid(): return QtCore.QVariant()
        if index.row() < 0 or index.row() >= self.cameraList.itemCount():
            return QtCore.QVariant()

        if role == QtCore.Qt.DisplayRole:
            return QtCore.QVariant(self.cameraList.items[index.row()].descr)
        elif role == QtCore.Qt.DecorationRole:
            return QtCore.QVariant(self.cameraList.items[index.row()].preview)

        return QtCore.QVariant()

class CCameraChooserDlg(QtGui.QDialog):
    def __init__(self):
        self.selected = None

        QtGui.QDialog.__init__(self)
        self.ui = uiSelectCamera.Ui_SelectCameraDlg()
        self.ui.setupUi(self)

        self._cameraModel = CCameraListModel()
        self._initialized = False
        self.ui.lsvCameras.setModel(self._cameraModel)

        self.connect(self.ui.lsvCameras.selectionModel(),
            QtCore.SIGNAL("selectionChanged(const QItemSelection &, const QItemSelection &)"),
            self.onSelectionChanged)

        self.btok = self.ui.buttonBox.button(QtGui.QDialogButtonBox.Ok)
        self.updateActions()

    def updateActions(self):
        if self.btok != None:
            self.btok.setEnabled(self.getSelectedItem() != None)

    def refreshList(self):
        QtGui.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        self._cameraModel.refreshList()
        QtGui.QApplication.restoreOverrideCursor()

    def onSelectionChanged(self, selAdded, selRemoved):
        self.updateActions()

    def on_btRefresh_clicked(self, valid=True):
        if not valid: return
        self.refreshList()
        self.updateActions()

    def getSelectedItem(self):
        it = self.ui.lsvCameras.selectionModel()
        sel = [ self._cameraModel.cameraList.items[r.row()] for r in it.selectedIndexes() ]
        if len(sel) > 0: return sel[0]
        return None

    def accept(self):
        self.selected = self.getSelectedItem()
        if self.selected == None: self.done(0)
        else:
            # print self.selected.descr
            self.done(1)

    def reject(self):
        self.done(0)

    def showEvent(self, event):
        if not self._initialized:
            self.refreshList()
            self._initialized = True

def testChooser():
    app = QtGui.QApplication(sys.argv)
    myapp = CCameraChooserDlg()
    myapp.show()
    sys.exit(app.exec_())
    pass

def testCameraList():
    C = CCameraList()
    C.refreshList()

if __name__=="__main__":
    testChooser()
