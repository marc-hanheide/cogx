# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'selectCamera.ui'
#
# Created: Tue Aug 18 09:57:58 2009
#      by: PyQt4 UI code generator 4.4.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

class Ui_SelectCameraDlg(object):
    def setupUi(self, SelectCameraDlg):
        SelectCameraDlg.setObjectName("SelectCameraDlg")
        SelectCameraDlg.resize(640, 480)
        self.verticalLayout_2 = QtGui.QVBoxLayout(SelectCameraDlg)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.widget_2 = QtGui.QWidget(SelectCameraDlg)
        self.widget_2.setObjectName("widget_2")
        self.horizontalLayout = QtGui.QHBoxLayout(self.widget_2)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.lsvCameras = QtGui.QListView(self.widget_2)
        self.lsvCameras.setObjectName("lsvCameras")
        self.horizontalLayout.addWidget(self.lsvCameras)
        self.widget = QtGui.QWidget(self.widget_2)
        self.widget.setObjectName("widget")
        self.verticalLayout = QtGui.QVBoxLayout(self.widget)
        self.verticalLayout.setObjectName("verticalLayout")
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.btRefresh = QtGui.QPushButton(self.widget)
        self.btRefresh.setObjectName("btRefresh")
        self.verticalLayout.addWidget(self.btRefresh)
        spacerItem1 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem1)
        self.horizontalLayout.addWidget(self.widget)
        self.verticalLayout_2.addWidget(self.widget_2)
        self.buttonBox = QtGui.QDialogButtonBox(SelectCameraDlg)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtGui.QDialogButtonBox.Cancel|QtGui.QDialogButtonBox.Ok)
        self.buttonBox.setObjectName("buttonBox")
        self.verticalLayout_2.addWidget(self.buttonBox)

        self.retranslateUi(SelectCameraDlg)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL("accepted()"), SelectCameraDlg.accept)
        QtCore.QObject.connect(self.buttonBox, QtCore.SIGNAL("rejected()"), SelectCameraDlg.reject)
        QtCore.QMetaObject.connectSlotsByName(SelectCameraDlg)

    def retranslateUi(self, SelectCameraDlg):
        SelectCameraDlg.setWindowTitle(QtGui.QApplication.translate("SelectCameraDlg", "Select Camera", None, QtGui.QApplication.UnicodeUTF8))
        self.btRefresh.setText(QtGui.QApplication.translate("SelectCameraDlg", "Refresh", None, QtGui.QApplication.UnicodeUTF8))


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    SelectCameraDlg = QtGui.QDialog()
    ui = Ui_SelectCameraDlg()
    ui.setupUi(SelectCameraDlg)
    SelectCameraDlg.show()
    sys.exit(app.exec_())

