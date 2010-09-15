#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim

import os, sys, time
import re
import tempfile
from PyQt4 import QtCore, QtGui

from qtui import uitextedit

class CTextEditor(QtGui.QDialog):
    def __init__(self, parent):
        QtGui.QDialog.__init__(self, parent)
        self.ui = uitextedit.Ui_SimpleTextEditor()
        self.ui.setupUi(self)

        self.connect(self.ui.buttonBox, QtCore.SIGNAL("accepted()"), self.onEditSave)
        self.connect(self.ui.buttonBox, QtCore.SIGNAL("rejected()"), self.onEditCancel)

        self.filename = None

    # Dialog entry point
    def editFile(self, filename, line=None):
        self.filename = None
        try:
            f = open(filename)
            text = f.read()
            f.close()
        except:
            text = ""
        editor = self.ui.textEdit
        editor.setPlainText(text)
        doc = editor.document()
        doc.setModified(False)
        self.filename = filename
        self.setWindowTitle(os.path.basename(filename))
        if line != None:
            cursor = editor.textCursor()
            cursor.movePosition(QtGui.QTextCursor.Down, QtGui.QTextCursor.MoveAnchor, line)
            editor.setTextCursor(cursor)

        rv = self.exec_()

    def onEditSave(self):
        editor = self.ui.textEdit
        text = editor.toPlainText()
        f = open(self.filename, "w")
        f.write(text)
        f.close()
        doc = editor.document()
        doc.setModified(False)
        self.done(QtGui.QDialog.Accepted)

    def maybeSave(self):
        editor = self.ui.textEdit
        doc = editor.document()
        if doc.isModified():
            print "TODO> Warning, modified"
            return False
        return True

    def onEditCancel(self):
        self.maybeSave()
        self.done(QtGui.QDialog.Rejected)

    def closeEvent(self, event):
        if self.maybeSave(): event.accept()
        else: event.cancel()

