#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim

import os, sys, time
import re
import tempfile
from PyQt4 import QtCore, QtGui

from qtui import uiselectcomponentsdlg

class CSelectComponentsDlg(QtGui.QDialog):
    def __init__(self, parent):
        QtGui.QDialog.__init__(self, parent)
        self.ui = uiselectcomponentsdlg.Ui_DlgSelectComponents()
        self.ui.setupUi(self)

        self.components = None

    def setComponentList(self, components):
        self.components = components
        self.ui.listWidget.clear()
        for c in components:
            s = "%s (%s %s)" % (c.cid, c.subarch, c.lang)
            w = QtGui.QListWidgetItem(s, self.ui.listWidget)
            w.setFlags(w.flags() | QtCore.Qt.ItemIsUserCheckable)
            w.setData(QtCore.Qt.CheckStateRole, QtCore.QVariant(QtCore.Qt.Checked))
            w.setData(QtCore.Qt.UserRole, c)
            w.setCheckState(2 if c.status else 0)

    def accept(self):
        wdgts = self.ui.listWidget.findItems("", QtCore.Qt.MatchContains)
        for w in wdgts:
            c = w.data(QtCore.Qt.UserRole).toPyObject()
            c.status = w.checkState()
        self.done(QtGui.QDialog.Accepted)
