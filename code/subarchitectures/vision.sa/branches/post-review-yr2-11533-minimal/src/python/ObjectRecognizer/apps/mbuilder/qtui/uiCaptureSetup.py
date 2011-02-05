# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'captureSetup.ui'
#
# Created: Wed May 26 11:36:19 2010
#      by: PyQt4 UI code generator 4.6
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

class Ui_dlgCaptureSetup(object):
    def setupUi(self, dlgCaptureSetup):
        dlgCaptureSetup.setObjectName("dlgCaptureSetup")
        dlgCaptureSetup.resize(384, 126)
        self.verticalLayout = QtGui.QVBoxLayout(dlgCaptureSetup)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtGui.QLabel(dlgCaptureSetup)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.txtLambdaStep = QtGui.QLineEdit(dlgCaptureSetup)
        self.txtLambdaStep.setMaximumSize(QtCore.QSize(80, 16777215))
        self.txtLambdaStep.setObjectName("txtLambdaStep")
        self.horizontalLayout.addWidget(self.txtLambdaStep)
        self.ckReverse = QtGui.QCheckBox(dlgCaptureSetup)
        self.ckReverse.setObjectName("ckReverse")
        self.horizontalLayout.addWidget(self.ckReverse)
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_2 = QtGui.QLabel(dlgCaptureSetup)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_2.addWidget(self.label_2)
        self.txtPhiList = QtGui.QLineEdit(dlgCaptureSetup)
        self.txtPhiList.setObjectName("txtPhiList")
        self.horizontalLayout_2.addWidget(self.txtPhiList)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        spacerItem1 = QtGui.QSpacerItem(20, 30, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem1)
        self.buttonBox = QtGui.QDialogButtonBox(dlgCaptureSetup)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtGui.QDialogButtonBox.Cancel|QtGui.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.verticalLayout.addWidget(self.buttonBox)
        self.label.setBuddy(self.txtLambdaStep)
        self.label_2.setBuddy(self.txtPhiList)

        self.retranslateUi(dlgCaptureSetup)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL("accepted()"), dlgCaptureSetup.accept)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL("rejected()"), dlgCaptureSetup.reject)
        QtCore.QMetaObject.connectSlotsByName(dlgCaptureSetup)
        dlgCaptureSetup.setTabOrder(self.txtLambdaStep, self.txtPhiList)
        dlgCaptureSetup.setTabOrder(self.txtPhiList, self.buttonBox)

    def retranslateUi(self, dlgCaptureSetup):
        dlgCaptureSetup.setWindowTitle(QtGui.QApplication.translate("dlgCaptureSetup", "Setup Capture", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("dlgCaptureSetup", "Lambda step:", None, QtGui.QApplication.UnicodeUTF8))
        self.txtLambdaStep.setToolTip(QtGui.QApplication.translate("dlgCaptureSetup", "Rotation of the object between views (in degrees)", None, QtGui.QApplication.UnicodeUTF8))
        self.txtLambdaStep.setInputMask(QtGui.QApplication.translate("dlgCaptureSetup", "00; ", None, QtGui.QApplication.UnicodeUTF8))
        self.txtLambdaStep.setText(QtGui.QApplication.translate("dlgCaptureSetup", "45", None, QtGui.QApplication.UnicodeUTF8))
        self.ckReverse.setText(QtGui.QApplication.translate("dlgCaptureSetup", "Reverse", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("dlgCaptureSetup", "Default Phis:", None, QtGui.QApplication.UnicodeUTF8))
        self.txtPhiList.setToolTip(QtGui.QApplication.translate("dlgCaptureSetup", "A comma separated list of elevation angles (in degrees)", None, QtGui.QApplication.UnicodeUTF8))
        self.txtPhiList.setText(QtGui.QApplication.translate("dlgCaptureSetup", "0, 15, 30, 45", None, QtGui.QApplication.UnicodeUTF8))


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    dlgCaptureSetup = QtGui.QDialog()
    ui = Ui_dlgCaptureSetup()
    ui.setupUi(dlgCaptureSetup)
    dlgCaptureSetup.show()
    sys.exit(app.exec_())

