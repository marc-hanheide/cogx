# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'selectcomponentsdlg.ui'
#
# Created: Tue Jun  1 21:03:00 2010
#      by: PyQt4 UI code generator 4.7.2
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

class Ui_DlgSelectComponents(object):
    def setupUi(self, DlgSelectComponents):
        DlgSelectComponents.setObjectName("DlgSelectComponents")
        DlgSelectComponents.setWindowModality(QtCore.Qt.ApplicationModal)
        DlgSelectComponents.resize(400, 399)
        self.verticalLayout = QtGui.QVBoxLayout(DlgSelectComponents)
        self.verticalLayout.setObjectName("verticalLayout")
        self.listWidget = QtGui.QListWidget(DlgSelectComponents)
        self.listWidget.setAlternatingRowColors(True)
        self.listWidget.setSelectionMode(QtGui.QAbstractItemView.SingleSelection)
        self.listWidget.setSelectionBehavior(QtGui.QAbstractItemView.SelectItems)
        self.listWidget.setObjectName("listWidget")
        self.verticalLayout.addWidget(self.listWidget)
        self.buttonBox = QtGui.QDialogButtonBox(DlgSelectComponents)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtGui.QDialogButtonBox.Cancel|QtGui.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.verticalLayout.addWidget(self.buttonBox)

        self.retranslateUi(DlgSelectComponents)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL("accepted()"), DlgSelectComponents.accept)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL("rejected()"), DlgSelectComponents.reject)
        QtCore.QMetaObject.connectSlotsByName(DlgSelectComponents)

    def retranslateUi(self, DlgSelectComponents):
        DlgSelectComponents.setWindowTitle(QtGui.QApplication.translate("DlgSelectComponents", "Select Components", None, QtGui.QApplication.UnicodeUTF8))

