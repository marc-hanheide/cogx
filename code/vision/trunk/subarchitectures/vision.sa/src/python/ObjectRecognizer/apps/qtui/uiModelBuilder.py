# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'modelBuilder.ui'
#
# Created: Fri Sep 11 13:59:16 2009
#      by: PyQt4 UI code generator 4.4.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui
from qwCameraPlacement import CQwCameraPlacement

class Ui_ModelBuilder(object):
    def setupUi(self, ModelBuilder):
        ModelBuilder.setObjectName("ModelBuilder")
        ModelBuilder.resize(711, 606)
        self.centralwidget = QtGui.QWidget(ModelBuilder)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout = QtGui.QVBoxLayout(self.centralwidget)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout_5 = QtGui.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.scrollArea = QtGui.QScrollArea(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.scrollArea.sizePolicy().hasHeightForWidth())
        self.scrollArea.setSizePolicy(sizePolicy)
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setObjectName("scrollArea")
        self.scrollAreaWidgetContents = QtGui.QWidget(self.scrollArea)
        self.scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 445, 403))
        self.scrollAreaWidgetContents.setObjectName("scrollAreaWidgetContents")
        self.cameraView = QtGui.QLabel(self.scrollAreaWidgetContents)
        self.cameraView.setGeometry(QtCore.QRect(10, 10, 31, 41))
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Ignored, QtGui.QSizePolicy.Ignored)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(1)
        sizePolicy.setHeightForWidth(self.cameraView.sizePolicy().hasHeightForWidth())
        self.cameraView.setSizePolicy(sizePolicy)
        self.cameraView.setObjectName("cameraView")
        self.scrollArea.setWidget(self.scrollAreaWidgetContents)
        self.horizontalLayout_5.addWidget(self.scrollArea)
        self.widget = QtGui.QWidget(self.centralwidget)
        self.widget.setMinimumSize(QtCore.QSize(200, 0))
        self.widget.setObjectName("widget")
        self.verticalLayout_4 = QtGui.QVBoxLayout(self.widget)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.frame = QtGui.QFrame(self.widget)
        self.frame.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtGui.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.verticalLayout_2 = QtGui.QVBoxLayout(self.frame)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_2 = QtGui.QLabel(self.frame)
        self.label_2.setObjectName("label_2")
        self.verticalLayout_2.addWidget(self.label_2)
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.btStartCamera = QtGui.QPushButton(self.frame)
        self.btStartCamera.setObjectName("btStartCamera")
        self.horizontalLayout_4.addWidget(self.btStartCamera)
        self.btStopCamera = QtGui.QPushButton(self.frame)
        self.btStopCamera.setObjectName("btStopCamera")
        self.horizontalLayout_4.addWidget(self.btStopCamera)
        self.btSelectCamera = QtGui.QPushButton(self.frame)
        self.btSelectCamera.setObjectName("btSelectCamera")
        self.horizontalLayout_4.addWidget(self.btSelectCamera)
        self.verticalLayout_2.addLayout(self.horizontalLayout_4)
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.btZoomIn = QtGui.QPushButton(self.frame)
        self.btZoomIn.setObjectName("btZoomIn")
        self.horizontalLayout_2.addWidget(self.btZoomIn)
        self.btZoomOut = QtGui.QPushButton(self.frame)
        self.btZoomOut.setObjectName("btZoomOut")
        self.horizontalLayout_2.addWidget(self.btZoomOut)
        spacerItem = QtGui.QSpacerItem(20, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem)
        self.verticalLayout_2.addLayout(self.horizontalLayout_2)
        self.verticalLayout_4.addWidget(self.frame)
        self.horizontalLayout_6 = QtGui.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.ckCalcSift = QtGui.QCheckBox(self.widget)
        self.ckCalcSift.setObjectName("ckCalcSift")
        self.horizontalLayout_6.addWidget(self.ckCalcSift)
        spacerItem1 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_6.addItem(spacerItem1)
        self.verticalLayout_4.addLayout(self.horizontalLayout_6)
        spacerItem2 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout_4.addItem(spacerItem2)
        self.frame_3 = QtGui.QFrame(self.widget)
        self.frame_3.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtGui.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.verticalLayout_5 = QtGui.QVBoxLayout(self.frame_3)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.wwCameraPlacement = CQwCameraPlacement(self.frame_3)
        self.wwCameraPlacement.setMinimumSize(QtCore.QSize(0, 120))
        self.wwCameraPlacement.setObjectName("wwCameraPlacement")
        self.verticalLayout_5.addWidget(self.wwCameraPlacement)
        self.verticalLayout_4.addWidget(self.frame_3)
        self.frame_2 = QtGui.QFrame(self.widget)
        self.frame_2.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtGui.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.verticalLayout_3 = QtGui.QVBoxLayout(self.frame_2)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.horizontalLayout_8 = QtGui.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.label_4 = QtGui.QLabel(self.frame_2)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_8.addWidget(self.label_4)
        spacerItem3 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_8.addItem(spacerItem3)
        self.btCaptureSetup = QtGui.QPushButton(self.frame_2)
        self.btCaptureSetup.setObjectName("btCaptureSetup")
        self.horizontalLayout_8.addWidget(self.btCaptureSetup)
        self.verticalLayout_3.addLayout(self.horizontalLayout_8)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.btSaveView = QtGui.QPushButton(self.frame_2)
        self.btSaveView.setObjectName("btSaveView")
        self.horizontalLayout_3.addWidget(self.btSaveView)
        spacerItem4 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem4)
        self.verticalLayout_3.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_7 = QtGui.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.label_3 = QtGui.QLabel(self.frame_2)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_7.addWidget(self.label_3)
        self.txtLambda = QtGui.QLineEdit(self.frame_2)
        self.txtLambda.setMinimumSize(QtCore.QSize(0, 0))
        self.txtLambda.setMaximumSize(QtCore.QSize(64, 16777215))
        self.txtLambda.setMaxLength(3)
        self.txtLambda.setObjectName("txtLambda")
        self.horizontalLayout_7.addWidget(self.txtLambda)
        self.btNextView = QtGui.QPushButton(self.frame_2)
        self.btNextView.setObjectName("btNextView")
        self.horizontalLayout_7.addWidget(self.btNextView)
        spacerItem5 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_7.addItem(spacerItem5)
        self.verticalLayout_3.addLayout(self.horizontalLayout_7)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtGui.QLabel(self.frame_2)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.cbElevation = QtGui.QComboBox(self.frame_2)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.cbElevation.sizePolicy().hasHeightForWidth())
        self.cbElevation.setSizePolicy(sizePolicy)
        self.cbElevation.setEditable(False)
        self.cbElevation.setObjectName("cbElevation")
        self.horizontalLayout.addWidget(self.cbElevation)
        spacerItem6 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem6)
        self.verticalLayout_3.addLayout(self.horizontalLayout)
        self.verticalLayout_4.addWidget(self.frame_2)
        self.horizontalLayout_5.addWidget(self.widget)
        self.verticalLayout.addLayout(self.horizontalLayout_5)
        self.lsvImages = QtGui.QListView(self.centralwidget)
        self.lsvImages.setEnabled(True)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lsvImages.sizePolicy().hasHeightForWidth())
        self.lsvImages.setSizePolicy(sizePolicy)
        self.lsvImages.setMinimumSize(QtCore.QSize(0, 140))
        self.lsvImages.setMaximumSize(QtCore.QSize(16777215, 160))
        self.lsvImages.setIconSize(QtCore.QSize(80, 60))
        self.lsvImages.setProperty("isWrapping", QtCore.QVariant(False))
        self.lsvImages.setSpacing(4)
        self.lsvImages.setViewMode(QtGui.QListView.IconMode)
        self.lsvImages.setObjectName("lsvImages")
        self.verticalLayout.addWidget(self.lsvImages)
        ModelBuilder.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(ModelBuilder)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 711, 23))
        self.menubar.setObjectName("menubar")
        self.menuModel = QtGui.QMenu(self.menubar)
        self.menuModel.setObjectName("menuModel")
        self.menuCamera = QtGui.QMenu(self.menubar)
        self.menuCamera.setEnabled(True)
        self.menuCamera.setObjectName("menuCamera")
        self.menuBuild = QtGui.QMenu(self.menubar)
        self.menuBuild.setObjectName("menuBuild")
        ModelBuilder.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(ModelBuilder)
        self.statusbar.setObjectName("statusbar")
        ModelBuilder.setStatusBar(self.statusbar)
        self.actionOpen = QtGui.QAction(ModelBuilder)
        self.actionOpen.setObjectName("actionOpen")
        self.actionNew = QtGui.QAction(ModelBuilder)
        self.actionNew.setObjectName("actionNew")
        self.actionClose = QtGui.QAction(ModelBuilder)
        self.actionClose.setObjectName("actionClose")
        self.actionQuit = QtGui.QAction(ModelBuilder)
        self.actionQuit.setObjectName("actionQuit")
        self.actionSelectCamera = QtGui.QAction(ModelBuilder)
        self.actionSelectCamera.setObjectName("actionSelectCamera")
        self.actionStartCamera = QtGui.QAction(ModelBuilder)
        self.actionStartCamera.setObjectName("actionStartCamera")
        self.actionStopCamera = QtGui.QAction(ModelBuilder)
        self.actionStopCamera.setObjectName("actionStopCamera")
        self.actionLoadElevationPreviews = QtGui.QAction(ModelBuilder)
        self.actionLoadElevationPreviews.setObjectName("actionLoadElevationPreviews")
        self.actCaptureSetup = QtGui.QAction(ModelBuilder)
        self.actCaptureSetup.setObjectName("actCaptureSetup")
        self.actNextLambda = QtGui.QAction(ModelBuilder)
        self.actNextLambda.setObjectName("actNextLambda")
        self.actSaveView = QtGui.QAction(ModelBuilder)
        self.actSaveView.setObjectName("actSaveView")
        self.actUpdateModel = QtGui.QAction(ModelBuilder)
        self.actUpdateModel.setObjectName("actUpdateModel")
        self.actRebuildModel = QtGui.QAction(ModelBuilder)
        self.actRebuildModel.setObjectName("actRebuildModel")
        self.menuModel.addAction(self.actionOpen)
        self.menuModel.addAction(self.actionNew)
        self.menuModel.addAction(self.actionClose)
        self.menuModel.addSeparator()
        self.menuModel.addAction(self.actionQuit)
        self.menuCamera.addAction(self.actionSelectCamera)
        self.menuCamera.addAction(self.actionStartCamera)
        self.menuCamera.addAction(self.actionStopCamera)
        self.menuBuild.addAction(self.actUpdateModel)
        self.menuBuild.addAction(self.actRebuildModel)
        self.menubar.addAction(self.menuModel.menuAction())
        self.menubar.addAction(self.menuCamera.menuAction())
        self.menubar.addAction(self.menuBuild.menuAction())
        self.label_3.setBuddy(self.txtLambda)
        self.label.setBuddy(self.cbElevation)

        self.retranslateUi(ModelBuilder)
        QtCore.QObject.connect(self.btStartCamera, QtCore.SIGNAL("clicked()"), self.actionStartCamera.trigger)
        QtCore.QObject.connect(self.btStopCamera, QtCore.SIGNAL("clicked()"), self.actionStopCamera.trigger)
        QtCore.QObject.connect(self.btSelectCamera, QtCore.SIGNAL("clicked()"), self.actionSelectCamera.trigger)
        QtCore.QObject.connect(self.cbElevation, QtCore.SIGNAL("currentIndexChanged(int)"), self.actionLoadElevationPreviews.trigger)
        QtCore.QObject.connect(self.btCaptureSetup, QtCore.SIGNAL("clicked()"), self.actCaptureSetup.trigger)
        QtCore.QObject.connect(self.btNextView, QtCore.SIGNAL("clicked()"), self.actNextLambda.trigger)
        QtCore.QObject.connect(self.btSaveView, QtCore.SIGNAL("clicked()"), self.actSaveView.trigger)
        QtCore.QMetaObject.connectSlotsByName(ModelBuilder)

    def retranslateUi(self, ModelBuilder):
        ModelBuilder.setWindowTitle(QtGui.QApplication.translate("ModelBuilder", "Model Builder", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("ModelBuilder", "Camera", None, QtGui.QApplication.UnicodeUTF8))
        self.btStartCamera.setText(QtGui.QApplication.translate("ModelBuilder", "Start", None, QtGui.QApplication.UnicodeUTF8))
        self.btStopCamera.setText(QtGui.QApplication.translate("ModelBuilder", "Stop", None, QtGui.QApplication.UnicodeUTF8))
        self.btSelectCamera.setText(QtGui.QApplication.translate("ModelBuilder", "Select...", None, QtGui.QApplication.UnicodeUTF8))
        self.btZoomIn.setText(QtGui.QApplication.translate("ModelBuilder", "+", None, QtGui.QApplication.UnicodeUTF8))
        self.btZoomOut.setText(QtGui.QApplication.translate("ModelBuilder", "-", None, QtGui.QApplication.UnicodeUTF8))
        self.ckCalcSift.setText(QtGui.QApplication.translate("ModelBuilder", "Calculate SIFT", None, QtGui.QApplication.UnicodeUTF8))
        self.label_4.setText(QtGui.QApplication.translate("ModelBuilder", "Capture", None, QtGui.QApplication.UnicodeUTF8))
        self.btCaptureSetup.setText(QtGui.QApplication.translate("ModelBuilder", "Setup...", None, QtGui.QApplication.UnicodeUTF8))
        self.btSaveView.setText(QtGui.QApplication.translate("ModelBuilder", "Save View", None, QtGui.QApplication.UnicodeUTF8))
        self.label_3.setText(QtGui.QApplication.translate("ModelBuilder", "Lambda:", None, QtGui.QApplication.UnicodeUTF8))
        self.txtLambda.setInputMask(QtGui.QApplication.translate("ModelBuilder", "000; ", None, QtGui.QApplication.UnicodeUTF8))
        self.txtLambda.setText(QtGui.QApplication.translate("ModelBuilder", "0", None, QtGui.QApplication.UnicodeUTF8))
        self.btNextView.setText(QtGui.QApplication.translate("ModelBuilder", "Next", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("ModelBuilder", "Phi:", None, QtGui.QApplication.UnicodeUTF8))
        self.menuModel.setTitle(QtGui.QApplication.translate("ModelBuilder", "&Model", None, QtGui.QApplication.UnicodeUTF8))
        self.menuCamera.setTitle(QtGui.QApplication.translate("ModelBuilder", "&Camera", None, QtGui.QApplication.UnicodeUTF8))
        self.menuBuild.setTitle(QtGui.QApplication.translate("ModelBuilder", "&Build", None, QtGui.QApplication.UnicodeUTF8))
        self.actionOpen.setText(QtGui.QApplication.translate("ModelBuilder", "Open", None, QtGui.QApplication.UnicodeUTF8))
        self.actionNew.setText(QtGui.QApplication.translate("ModelBuilder", "New", None, QtGui.QApplication.UnicodeUTF8))
        self.actionClose.setText(QtGui.QApplication.translate("ModelBuilder", "Close", None, QtGui.QApplication.UnicodeUTF8))
        self.actionQuit.setText(QtGui.QApplication.translate("ModelBuilder", "&Quit", None, QtGui.QApplication.UnicodeUTF8))
        self.actionQuit.setToolTip(QtGui.QApplication.translate("ModelBuilder", "Quit", None, QtGui.QApplication.UnicodeUTF8))
        self.actionQuit.setShortcut(QtGui.QApplication.translate("ModelBuilder", "Ctrl+Q", None, QtGui.QApplication.UnicodeUTF8))
        self.actionSelectCamera.setText(QtGui.QApplication.translate("ModelBuilder", "&Select ...", None, QtGui.QApplication.UnicodeUTF8))
        self.actionStartCamera.setText(QtGui.QApplication.translate("ModelBuilder", "S&tart", None, QtGui.QApplication.UnicodeUTF8))
        self.actionStopCamera.setText(QtGui.QApplication.translate("ModelBuilder", "Sto&p", None, QtGui.QApplication.UnicodeUTF8))
        self.actionLoadElevationPreviews.setText(QtGui.QApplication.translate("ModelBuilder", "Load Elevation Previews", None, QtGui.QApplication.UnicodeUTF8))
        self.actCaptureSetup.setText(QtGui.QApplication.translate("ModelBuilder", "Capture Setup...", None, QtGui.QApplication.UnicodeUTF8))
        self.actNextLambda.setText(QtGui.QApplication.translate("ModelBuilder", "Next Lambda", None, QtGui.QApplication.UnicodeUTF8))
        self.actSaveView.setText(QtGui.QApplication.translate("ModelBuilder", "Save view", None, QtGui.QApplication.UnicodeUTF8))
        self.actUpdateModel.setText(QtGui.QApplication.translate("ModelBuilder", "&Update model", None, QtGui.QApplication.UnicodeUTF8))
        self.actUpdateModel.setToolTip(QtGui.QApplication.translate("ModelBuilder", "Buid the model from current set of images", None, QtGui.QApplication.UnicodeUTF8))
        self.actRebuildModel.setText(QtGui.QApplication.translate("ModelBuilder", "&Rebuild model", None, QtGui.QApplication.UnicodeUTF8))


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    ModelBuilder = QtGui.QMainWindow()
    ui = Ui_ModelBuilder()
    ui.setupUi(ModelBuilder)
    ModelBuilder.show()
    sys.exit(app.exec_())

