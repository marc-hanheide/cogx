# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'textedit.ui'
#
# Created: Wed Sep 15 18:26:11 2010
#      by: PyQt4 UI code generator 4.6
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

class Ui_SimpleTextEditor(object):
    def setupUi(self, SimpleTextEditor):
        SimpleTextEditor.setObjectName("SimpleTextEditor")
        SimpleTextEditor.resize(531, 383)
        self.verticalLayout = QtGui.QVBoxLayout(SimpleTextEditor)
        self.verticalLayout.setObjectName("verticalLayout")
        self.textEdit = QtGui.QTextEdit(SimpleTextEditor)
        self.textEdit.setAcceptRichText(False)
        self.textEdit.setObjectName("textEdit")
        self.verticalLayout.addWidget(self.textEdit)
        self.buttonBox = QtGui.QDialogButtonBox(SimpleTextEditor)
        self.buttonBox.setStandardButtons(QtGui.QDialogButtonBox.Cancel|QtGui.QDialogButtonBox.Save)
        self.buttonBox.setObjectName("buttonBox")
        self.verticalLayout.addWidget(self.buttonBox)

        self.retranslateUi(SimpleTextEditor)
        QtCore.QMetaObject.connectSlotsByName(SimpleTextEditor)

    def retranslateUi(self, SimpleTextEditor):
        SimpleTextEditor.setWindowTitle(QtGui.QApplication.translate("SimpleTextEditor", "Internal Editor", None, QtGui.QApplication.UnicodeUTF8))

