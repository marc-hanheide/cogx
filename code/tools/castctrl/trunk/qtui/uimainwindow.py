# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainwindow.ui'
#
# Created: Mon Jan 11 14:24:41 2010
#      by: PyQt4 UI code generator 4.6
#
# WARNING! All changes made in this file will be lost!
from cctextedit import CCastTextEdit

from PyQt4 import QtCore, QtGui

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(883, 668)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayout = QtGui.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.processTree = QtGui.QTreeView(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(1)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.processTree.sizePolicy().hasHeightForWidth())
        self.processTree.setSizePolicy(sizePolicy)
        self.processTree.setMinimumSize(QtCore.QSize(240, 0))
        self.processTree.setMaximumSize(QtCore.QSize(400, 16777215))
        self.processTree.setObjectName("processTree")
        self.horizontalLayout.addWidget(self.processTree)
        self.frame_6 = QtGui.QFrame(self.centralwidget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(2)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_6.sizePolicy().hasHeightForWidth())
        self.frame_6.setSizePolicy(sizePolicy)
        self.frame_6.setFrameShape(QtGui.QFrame.NoFrame)
        self.frame_6.setFrameShadow(QtGui.QFrame.Raised)
        self.frame_6.setObjectName("frame_6")
        self.verticalLayout_7 = QtGui.QVBoxLayout(self.frame_6)
        self.verticalLayout_7.setContentsMargins(0, 0, 0, -1)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.frame_5 = QtGui.QFrame(self.frame_6)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_5.sizePolicy().hasHeightForWidth())
        self.frame_5.setSizePolicy(sizePolicy)
        self.frame_5.setFrameShape(QtGui.QFrame.NoFrame)
        self.frame_5.setFrameShadow(QtGui.QFrame.Raised)
        self.frame_5.setObjectName("frame_5")
        self.horizontalLayout_5 = QtGui.QHBoxLayout(self.frame_5)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.frame = QtGui.QFrame(self.frame_5)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame.sizePolicy().hasHeightForWidth())
        self.frame.setSizePolicy(sizePolicy)
        self.frame.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtGui.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.verticalLayout_3 = QtGui.QVBoxLayout(self.frame)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.horizontalLayout_10 = QtGui.QHBoxLayout()
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.label = QtGui.QLabel(self.frame)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.label.setFont(font)
        self.label.setObjectName("label")
        self.horizontalLayout_10.addWidget(self.label)
        self.verticalLayout_3.addLayout(self.horizontalLayout_10)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.btServerStart = QtGui.QPushButton(self.frame)
        self.btServerStart.setObjectName("btServerStart")
        self.horizontalLayout_3.addWidget(self.btServerStart)
        self.btServerStop = QtGui.QPushButton(self.frame)
        self.btServerStop.setObjectName("btServerStop")
        self.horizontalLayout_3.addWidget(self.btServerStop)
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem)
        self.verticalLayout_3.addLayout(self.horizontalLayout_3)
        self.ckCleanupScript = QtGui.QCheckBox(self.frame)
        self.ckCleanupScript.setObjectName("ckCleanupScript")
        self.verticalLayout_3.addWidget(self.ckCleanupScript)
        spacerItem1 = QtGui.QSpacerItem(20, 8, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.MinimumExpanding)
        self.verticalLayout_3.addItem(spacerItem1)
        self.horizontalLayout_5.addWidget(self.frame)
        self.frame_2 = QtGui.QFrame(self.frame_5)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_2.sizePolicy().hasHeightForWidth())
        self.frame_2.setSizePolicy(sizePolicy)
        self.frame_2.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtGui.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.verticalLayout_4 = QtGui.QVBoxLayout(self.frame_2)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.horizontalLayout_9 = QtGui.QHBoxLayout()
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.label_2 = QtGui.QLabel(self.frame_2)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_9.addWidget(self.label_2)
        self.verticalLayout_4.addLayout(self.horizontalLayout_9)
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.btClientStart = QtGui.QPushButton(self.frame_2)
        self.btClientStart.setObjectName("btClientStart")
        self.horizontalLayout_4.addWidget(self.btClientStart)
        self.btClientStop = QtGui.QPushButton(self.frame_2)
        self.btClientStop.setObjectName("btClientStop")
        self.horizontalLayout_4.addWidget(self.btClientStop)
        spacerItem2 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem2)
        self.verticalLayout_4.addLayout(self.horizontalLayout_4)
        spacerItem3 = QtGui.QSpacerItem(20, 8, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.MinimumExpanding)
        self.verticalLayout_4.addItem(spacerItem3)
        self.horizontalLayout_5.addWidget(self.frame_2)
        self.frame_3 = QtGui.QFrame(self.frame_5)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.frame_3.sizePolicy().hasHeightForWidth())
        self.frame_3.setSizePolicy(sizePolicy)
        self.frame_3.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QtGui.QFrame.Raised)
        self.frame_3.setObjectName("frame_3")
        self.verticalLayout_5 = QtGui.QVBoxLayout(self.frame_3)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.horizontalLayout_7 = QtGui.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.label_9 = QtGui.QLabel(self.frame_3)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.label_9.setFont(font)
        self.label_9.setObjectName("label_9")
        self.horizontalLayout_7.addWidget(self.label_9)
        spacerItem4 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_7.addItem(spacerItem4)
        self.ckPeekabot = QtGui.QCheckBox(self.frame_3)
        self.ckPeekabot.setChecked(True)
        self.ckPeekabot.setObjectName("ckPeekabot")
        self.horizontalLayout_7.addWidget(self.ckPeekabot)
        self.verticalLayout_5.addLayout(self.horizontalLayout_7)
        self.horizontalLayout_8 = QtGui.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.btPlayerStart = QtGui.QPushButton(self.frame_3)
        self.btPlayerStart.setObjectName("btPlayerStart")
        self.horizontalLayout_8.addWidget(self.btPlayerStart)
        self.btPlayerStop = QtGui.QPushButton(self.frame_3)
        self.btPlayerStop.setObjectName("btPlayerStop")
        self.horizontalLayout_8.addWidget(self.btPlayerStop)
        spacerItem5 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_8.addItem(spacerItem5)
        self.verticalLayout_5.addLayout(self.horizontalLayout_8)
        spacerItem6 = QtGui.QSpacerItem(20, 8, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.MinimumExpanding)
        self.verticalLayout_5.addItem(spacerItem6)
        self.horizontalLayout_5.addWidget(self.frame_3)
        self.verticalLayout_7.addWidget(self.frame_5)
        self.tabWidget = QtGui.QTabWidget(self.frame_6)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(3)
        sizePolicy.setVerticalStretch(1)
        sizePolicy.setHeightForWidth(self.tabWidget.sizePolicy().hasHeightForWidth())
        self.tabWidget.setSizePolicy(sizePolicy)
        self.tabWidget.setObjectName("tabWidget")
        self.tabLogs = QtGui.QWidget()
        self.tabLogs.setObjectName("tabLogs")
        self.verticalLayout = QtGui.QVBoxLayout(self.tabLogs)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.logfileCmbx = QtGui.QComboBox(self.tabLogs)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(4)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.logfileCmbx.sizePolicy().hasHeightForWidth())
        self.logfileCmbx.setSizePolicy(sizePolicy)
        self.logfileCmbx.setMinimumSize(QtCore.QSize(200, 0))
        self.logfileCmbx.setObjectName("logfileCmbx")
        self.horizontalLayout_2.addWidget(self.logfileCmbx)
        self.btLogViewAll = QtGui.QPushButton(self.tabLogs)
        self.btLogViewAll.setObjectName("btLogViewAll")
        self.horizontalLayout_2.addWidget(self.btLogViewAll)
        self.btLogViewControl = QtGui.QPushButton(self.tabLogs)
        self.btLogViewControl.setObjectName("btLogViewControl")
        self.horizontalLayout_2.addWidget(self.btLogViewControl)
        spacerItem7 = QtGui.QSpacerItem(88, 17, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem7)
        self.btClearMainLog = QtGui.QPushButton(self.tabLogs)
        self.btClearMainLog.setObjectName("btClearMainLog")
        self.horizontalLayout_2.addWidget(self.btClearMainLog)
        self.ckShowFlushMsgs = QtGui.QCheckBox(self.tabLogs)
        self.ckShowFlushMsgs.setObjectName("ckShowFlushMsgs")
        self.horizontalLayout_2.addWidget(self.ckShowFlushMsgs)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.mainLogfileTxt = CCastTextEdit(self.tabLogs)
        self.mainLogfileTxt.setUndoRedoEnabled(False)
        self.mainLogfileTxt.setTextInteractionFlags(QtCore.Qt.TextSelectableByKeyboard|QtCore.Qt.TextSelectableByMouse)
        self.mainLogfileTxt.setObjectName("mainLogfileTxt")
        self.verticalLayout.addWidget(self.mainLogfileTxt)
        self.tabWidget.addTab(self.tabLogs, "")
        self.tab_2 = QtGui.QWidget()
        self.tab_2.setObjectName("tab_2")
        self.verticalLayout_2 = QtGui.QVBoxLayout(self.tab_2)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.frame_11 = QtGui.QFrame(self.tab_2)
        self.frame_11.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame_11.setFrameShadow(QtGui.QFrame.Raised)
        self.frame_11.setObjectName("frame_11")
        self.verticalLayout_14 = QtGui.QVBoxLayout(self.frame_11)
        self.verticalLayout_14.setObjectName("verticalLayout_14")
        self.label_3 = QtGui.QLabel(self.frame_11)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")
        self.verticalLayout_14.addWidget(self.label_3)
        self.horizontalLayout_13 = QtGui.QHBoxLayout()
        self.horizontalLayout_13.setObjectName("horizontalLayout_13")
        self.label_14 = QtGui.QLabel(self.frame_11)
        self.label_14.setObjectName("label_14")
        self.horizontalLayout_13.addWidget(self.label_14)
        self.clientHostCmbx = QtGui.QComboBox(self.frame_11)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.clientHostCmbx.sizePolicy().hasHeightForWidth())
        self.clientHostCmbx.setSizePolicy(sizePolicy)
        self.clientHostCmbx.setMinimumSize(QtCore.QSize(200, 0))
        self.clientHostCmbx.setObjectName("clientHostCmbx")
        self.horizontalLayout_13.addWidget(self.clientHostCmbx)
        spacerItem8 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_13.addItem(spacerItem8)
        self.verticalLayout_14.addLayout(self.horizontalLayout_13)
        self.horizontalLayout_14 = QtGui.QHBoxLayout()
        self.horizontalLayout_14.setObjectName("horizontalLayout_14")
        self.label_6 = QtGui.QLabel(self.frame_11)
        self.label_6.setObjectName("label_6")
        self.horizontalLayout_14.addWidget(self.label_6)
        self.clientConfigCmbx = QtGui.QComboBox(self.frame_11)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(4)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.clientConfigCmbx.sizePolicy().hasHeightForWidth())
        self.clientConfigCmbx.setSizePolicy(sizePolicy)
        self.clientConfigCmbx.setMinimumSize(QtCore.QSize(200, 0))
        self.clientConfigCmbx.setObjectName("clientConfigCmbx")
        self.horizontalLayout_14.addWidget(self.clientConfigCmbx)
        self.btBrowseCastFile = QtGui.QPushButton(self.frame_11)
        self.btBrowseCastFile.setObjectName("btBrowseCastFile")
        self.horizontalLayout_14.addWidget(self.btBrowseCastFile)
        self.btEditClientConfig = QtGui.QPushButton(self.frame_11)
        self.btEditClientConfig.setObjectName("btEditClientConfig")
        self.horizontalLayout_14.addWidget(self.btEditClientConfig)
        self.verticalLayout_14.addLayout(self.horizontalLayout_14)
        self.verticalLayout_2.addWidget(self.frame_11)
        self.frame_10 = QtGui.QFrame(self.tab_2)
        self.frame_10.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame_10.setFrameShadow(QtGui.QFrame.Raised)
        self.frame_10.setObjectName("frame_10")
        self.verticalLayout_13 = QtGui.QVBoxLayout(self.frame_10)
        self.verticalLayout_13.setObjectName("verticalLayout_13")
        self.label_12 = QtGui.QLabel(self.frame_10)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.label_12.setFont(font)
        self.label_12.setObjectName("label_12")
        self.verticalLayout_13.addWidget(self.label_12)
        self.horizontalLayout_11 = QtGui.QHBoxLayout()
        self.horizontalLayout_11.setObjectName("horizontalLayout_11")
        self.label_13 = QtGui.QLabel(self.frame_10)
        self.label_13.setObjectName("label_13")
        self.horizontalLayout_11.addWidget(self.label_13)
        self.playerHostCmbx = QtGui.QComboBox(self.frame_10)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.playerHostCmbx.sizePolicy().hasHeightForWidth())
        self.playerHostCmbx.setSizePolicy(sizePolicy)
        self.playerHostCmbx.setMinimumSize(QtCore.QSize(200, 0))
        self.playerHostCmbx.setObjectName("playerHostCmbx")
        self.horizontalLayout_11.addWidget(self.playerHostCmbx)
        spacerItem9 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_11.addItem(spacerItem9)
        self.verticalLayout_13.addLayout(self.horizontalLayout_11)
        self.horizontalLayout_12 = QtGui.QHBoxLayout()
        self.horizontalLayout_12.setObjectName("horizontalLayout_12")
        self.label_8 = QtGui.QLabel(self.frame_10)
        self.label_8.setObjectName("label_8")
        self.horizontalLayout_12.addWidget(self.label_8)
        self.playerConfigCmbx = QtGui.QComboBox(self.frame_10)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(4)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.playerConfigCmbx.sizePolicy().hasHeightForWidth())
        self.playerConfigCmbx.setSizePolicy(sizePolicy)
        self.playerConfigCmbx.setMinimumSize(QtCore.QSize(200, 0))
        self.playerConfigCmbx.setObjectName("playerConfigCmbx")
        self.horizontalLayout_12.addWidget(self.playerConfigCmbx)
        self.btBrowsePlayerFile = QtGui.QPushButton(self.frame_10)
        self.btBrowsePlayerFile.setObjectName("btBrowsePlayerFile")
        self.horizontalLayout_12.addWidget(self.btBrowsePlayerFile)
        self.btEditPlayerConfig = QtGui.QPushButton(self.frame_10)
        self.btEditPlayerConfig.setObjectName("btEditPlayerConfig")
        self.horizontalLayout_12.addWidget(self.btEditPlayerConfig)
        self.verticalLayout_13.addLayout(self.horizontalLayout_12)
        self.verticalLayout_2.addWidget(self.frame_10)
        self.frame_4 = QtGui.QFrame(self.tab_2)
        self.frame_4.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QtGui.QFrame.Raised)
        self.frame_4.setObjectName("frame_4")
        self.verticalLayout_8 = QtGui.QVBoxLayout(self.frame_4)
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.label_4 = QtGui.QLabel(self.frame_4)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.label_4.setFont(font)
        self.label_4.setObjectName("label_4")
        self.verticalLayout_8.addWidget(self.label_4)
        self.horizontalLayout_16 = QtGui.QHBoxLayout()
        self.horizontalLayout_16.setObjectName("horizontalLayout_16")
        self.label_10 = QtGui.QLabel(self.frame_4)
        self.label_10.setObjectName("label_10")
        self.horizontalLayout_16.addWidget(self.label_10)
        self.hostConfigCmbx = QtGui.QComboBox(self.frame_4)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(4)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.hostConfigCmbx.sizePolicy().hasHeightForWidth())
        self.hostConfigCmbx.setSizePolicy(sizePolicy)
        self.hostConfigCmbx.setMinimumSize(QtCore.QSize(200, 0))
        self.hostConfigCmbx.setObjectName("hostConfigCmbx")
        self.horizontalLayout_16.addWidget(self.hostConfigCmbx)
        self.btBrowseHostFile = QtGui.QPushButton(self.frame_4)
        self.btBrowseHostFile.setObjectName("btBrowseHostFile")
        self.horizontalLayout_16.addWidget(self.btBrowseHostFile)
        self.btEditHostConfig = QtGui.QPushButton(self.frame_4)
        self.btEditHostConfig.setObjectName("btEditHostConfig")
        self.horizontalLayout_16.addWidget(self.btEditHostConfig)
        self.verticalLayout_8.addLayout(self.horizontalLayout_16)
        self.verticalLayout_2.addWidget(self.frame_4)
        spacerItem10 = QtGui.QSpacerItem(20, 235, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem10)
        self.tabWidget.addTab(self.tab_2, "")
        self.tab_4 = QtGui.QWidget()
        self.tab_4.setObjectName("tab_4")
        self.verticalLayout_6 = QtGui.QVBoxLayout(self.tab_4)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.horizontalLayout_6 = QtGui.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.btBuild = QtGui.QPushButton(self.tab_4)
        self.btBuild.setObjectName("btBuild")
        self.horizontalLayout_6.addWidget(self.btBuild)
        self.btBuildInstall = QtGui.QPushButton(self.tab_4)
        self.btBuildInstall.setObjectName("btBuildInstall")
        self.horizontalLayout_6.addWidget(self.btBuildInstall)
        spacerItem11 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_6.addItem(spacerItem11)
        self.btCmakeGui = QtGui.QPushButton(self.tab_4)
        self.btCmakeGui.setObjectName("btCmakeGui")
        self.horizontalLayout_6.addWidget(self.btCmakeGui)
        self.verticalLayout_6.addLayout(self.horizontalLayout_6)
        self.buildLogfileTxt = CCastTextEdit(self.tab_4)
        self.buildLogfileTxt.setUndoRedoEnabled(False)
        self.buildLogfileTxt.setTextInteractionFlags(QtCore.Qt.TextSelectableByKeyboard|QtCore.Qt.TextSelectableByMouse)
        self.buildLogfileTxt.setObjectName("buildLogfileTxt")
        self.verticalLayout_6.addWidget(self.buildLogfileTxt)
        self.tabWidget.addTab(self.tab_4, "")
        self.verticalLayout_7.addWidget(self.tabWidget)
        self.horizontalLayout.addWidget(self.frame_6)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 883, 21))
        self.menubar.setObjectName("menubar")
        self.menuCast = QtGui.QMenu(self.menubar)
        self.menuCast.setObjectName("menuCast")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)
        self.actQuit = QtGui.QAction(MainWindow)
        self.actQuit.setObjectName("actQuit")
        self.actOpenClientConfig = QtGui.QAction(MainWindow)
        self.actOpenClientConfig.setObjectName("actOpenClientConfig")
        self.actOpenPlayerConfig = QtGui.QAction(MainWindow)
        self.actOpenPlayerConfig.setObjectName("actOpenPlayerConfig")
        self.actShowEnv = QtGui.QAction(MainWindow)
        self.actShowEnv.setObjectName("actShowEnv")
        self.actCtxShowBuildError = QtGui.QAction(MainWindow)
        self.actCtxShowBuildError.setObjectName("actCtxShowBuildError")
        self.actStartTerminal = QtGui.QAction(MainWindow)
        self.actStartTerminal.setObjectName("actStartTerminal")
        self.actOpenHostConfig = QtGui.QAction(MainWindow)
        self.actOpenHostConfig.setObjectName("actOpenHostConfig")
        self.menuCast.addAction(self.actShowEnv)
        self.menuCast.addSeparator()
        self.menuCast.addAction(self.actQuit)
        self.menuCast.addAction(self.actStartTerminal)
        self.menubar.addAction(self.menuCast.menuAction())

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QObject.connect(self.btBrowseCastFile, QtCore.SIGNAL("clicked()"), self.actOpenClientConfig.trigger)
        QtCore.QObject.connect(self.btBrowsePlayerFile, QtCore.SIGNAL("clicked()"), self.actOpenPlayerConfig.trigger)
        QtCore.QObject.connect(self.btBrowseHostFile, QtCore.SIGNAL("clicked()"), self.actOpenHostConfig.trigger)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QtGui.QApplication.translate("MainWindow", "CAST Control", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("MainWindow", "Servers", None, QtGui.QApplication.UnicodeUTF8))
        self.btServerStart.setText(QtGui.QApplication.translate("MainWindow", "Start", None, QtGui.QApplication.UnicodeUTF8))
        self.btServerStop.setText(QtGui.QApplication.translate("MainWindow", "Stop", None, QtGui.QApplication.UnicodeUTF8))
        self.ckCleanupScript.setText(QtGui.QApplication.translate("MainWindow", "Run cleanup script on start", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("MainWindow", "Client", None, QtGui.QApplication.UnicodeUTF8))
        self.btClientStart.setText(QtGui.QApplication.translate("MainWindow", "Start", None, QtGui.QApplication.UnicodeUTF8))
        self.btClientStop.setText(QtGui.QApplication.translate("MainWindow", "Stop", None, QtGui.QApplication.UnicodeUTF8))
        self.label_9.setText(QtGui.QApplication.translate("MainWindow", "Player", None, QtGui.QApplication.UnicodeUTF8))
        self.ckPeekabot.setText(QtGui.QApplication.translate("MainWindow", "Peekabot", None, QtGui.QApplication.UnicodeUTF8))
        self.btPlayerStart.setText(QtGui.QApplication.translate("MainWindow", "Start", None, QtGui.QApplication.UnicodeUTF8))
        self.btPlayerStop.setText(QtGui.QApplication.translate("MainWindow", "Stop", None, QtGui.QApplication.UnicodeUTF8))
        self.logfileCmbx.setStatusTip(QtGui.QApplication.translate("MainWindow", "Active log", None, QtGui.QApplication.UnicodeUTF8))
        self.btLogViewAll.setText(QtGui.QApplication.translate("MainWindow", "All", None, QtGui.QApplication.UnicodeUTF8))
        self.btLogViewControl.setText(QtGui.QApplication.translate("MainWindow", "Control", None, QtGui.QApplication.UnicodeUTF8))
        self.btClearMainLog.setText(QtGui.QApplication.translate("MainWindow", "Clear", None, QtGui.QApplication.UnicodeUTF8))
        self.ckShowFlushMsgs.setText(QtGui.QApplication.translate("MainWindow", "Show Flushed", None, QtGui.QApplication.UnicodeUTF8))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tabLogs), QtGui.QApplication.translate("MainWindow", "Logs", None, QtGui.QApplication.UnicodeUTF8))
        self.label_3.setText(QtGui.QApplication.translate("MainWindow", "Client", None, QtGui.QApplication.UnicodeUTF8))
        self.label_14.setText(QtGui.QApplication.translate("MainWindow", "Run on host", None, QtGui.QApplication.UnicodeUTF8))
        self.label_6.setText(QtGui.QApplication.translate("MainWindow", "Configuration", None, QtGui.QApplication.UnicodeUTF8))
        self.clientConfigCmbx.setStatusTip(QtGui.QApplication.translate("MainWindow", "CAST configuration file ", None, QtGui.QApplication.UnicodeUTF8))
        self.btBrowseCastFile.setText(QtGui.QApplication.translate("MainWindow", "...", None, QtGui.QApplication.UnicodeUTF8))
        self.btEditClientConfig.setText(QtGui.QApplication.translate("MainWindow", "Edit", None, QtGui.QApplication.UnicodeUTF8))
        self.label_12.setText(QtGui.QApplication.translate("MainWindow", "Player", None, QtGui.QApplication.UnicodeUTF8))
        self.label_13.setText(QtGui.QApplication.translate("MainWindow", "Run on host", None, QtGui.QApplication.UnicodeUTF8))
        self.label_8.setText(QtGui.QApplication.translate("MainWindow", "Configuration", None, QtGui.QApplication.UnicodeUTF8))
        self.playerConfigCmbx.setStatusTip(QtGui.QApplication.translate("MainWindow", "Player configuration file", None, QtGui.QApplication.UnicodeUTF8))
        self.btBrowsePlayerFile.setText(QtGui.QApplication.translate("MainWindow", "...", None, QtGui.QApplication.UnicodeUTF8))
        self.btEditPlayerConfig.setText(QtGui.QApplication.translate("MainWindow", "Edit", None, QtGui.QApplication.UnicodeUTF8))
        self.label_4.setText(QtGui.QApplication.translate("MainWindow", "Hosts", None, QtGui.QApplication.UnicodeUTF8))
        self.label_10.setText(QtGui.QApplication.translate("MainWindow", "Configuration", None, QtGui.QApplication.UnicodeUTF8))
        self.hostConfigCmbx.setStatusTip(QtGui.QApplication.translate("MainWindow", "CAST configuration file ", None, QtGui.QApplication.UnicodeUTF8))
        self.btBrowseHostFile.setText(QtGui.QApplication.translate("MainWindow", "...", None, QtGui.QApplication.UnicodeUTF8))
        self.btEditHostConfig.setText(QtGui.QApplication.translate("MainWindow", "Edit", None, QtGui.QApplication.UnicodeUTF8))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), QtGui.QApplication.translate("MainWindow", "Configure", None, QtGui.QApplication.UnicodeUTF8))
        self.btBuild.setText(QtGui.QApplication.translate("MainWindow", "make", None, QtGui.QApplication.UnicodeUTF8))
        self.btBuildInstall.setText(QtGui.QApplication.translate("MainWindow", "make install", None, QtGui.QApplication.UnicodeUTF8))
        self.btCmakeGui.setText(QtGui.QApplication.translate("MainWindow", "Config...", None, QtGui.QApplication.UnicodeUTF8))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_4), QtGui.QApplication.translate("MainWindow", "Build", None, QtGui.QApplication.UnicodeUTF8))
        self.menuCast.setTitle(QtGui.QApplication.translate("MainWindow", "&Cast", None, QtGui.QApplication.UnicodeUTF8))
        self.actQuit.setText(QtGui.QApplication.translate("MainWindow", "&Quit", None, QtGui.QApplication.UnicodeUTF8))
        self.actQuit.setShortcut(QtGui.QApplication.translate("MainWindow", "Ctrl+Q", None, QtGui.QApplication.UnicodeUTF8))
        self.actOpenClientConfig.setText(QtGui.QApplication.translate("MainWindow", "Select Client Configuration", None, QtGui.QApplication.UnicodeUTF8))
        self.actOpenPlayerConfig.setText(QtGui.QApplication.translate("MainWindow", "Select Player Configuration", None, QtGui.QApplication.UnicodeUTF8))
        self.actShowEnv.setText(QtGui.QApplication.translate("MainWindow", "Show ENV", None, QtGui.QApplication.UnicodeUTF8))
        self.actCtxShowBuildError.setText(QtGui.QApplication.translate("MainWindow", "Show code", None, QtGui.QApplication.UnicodeUTF8))
        self.actStartTerminal.setText(QtGui.QApplication.translate("MainWindow", "Start Terminal", None, QtGui.QApplication.UnicodeUTF8))
        self.actOpenHostConfig.setText(QtGui.QApplication.translate("MainWindow", "Select Host Configuration", None, QtGui.QApplication.UnicodeUTF8))

