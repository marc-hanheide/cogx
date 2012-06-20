#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim

from PyQt4 import QtCore, QtGui

class CCastTextEdit(QtGui.QTextEdit):
    def __init__(self, parent):
        QtGui.QTextEdit.__init__(self, parent)
        self.contextActions = []
        self.createActions()
        self.searchString = ""

    def contextMenuEvent(self, event):
        cur = self.textCursor()
        if not cur.hasSelection():
            cur = self.cursorForPosition(event.pos())
            self.setTextCursor(cur)
        menu = self.createStandardContextMenu()
        for act in self.contextActions:
            # TODO: if act.isValid(self)
            menu.addAction(act)
        action = menu.exec_(event.globalPos())

    def createActions(self):
        actFind = QtGui.QAction("Find...", self)
        actFind.setShortcutContext(QtCore.Qt.WidgetShortcut)
        #actFind.setShortcut("Ctrl+F") # XXX disabled, doesn't work
        self.connect(actFind, QtCore.SIGNAL("triggered()"), self.onFind)
        self.contextActions.append(actFind)

        actFindAgain = QtGui.QAction("Find again", self)
        actFind.setShortcutContext(QtCore.Qt.WidgetShortcut)
        #actFindAgain.setShortcut("Ctrl+G") # XXX disabled, doesn't work
        self.connect(actFindAgain, QtCore.SIGNAL("triggered()"), self.onFindAgain)
        self.contextActions.append(actFindAgain)

    def doFind(self, string):
        rv = self.find(string)
        if not rv:
            self.moveCursor(QtGui.QTextCursor.Start);
            rv = self.find(self.searchString)
        if not rv:
            reply = QtGui.QMessageBox.question(self, 'Message',
                    "'%s' not found." % string,
                    QtGui.QMessageBox.Ok, QtGui.QMessageBox.Ok)

    def onFind(self):
        text, ok = QtGui.QInputDialog.getText(self, "Find", "String:", QtGui.QLineEdit.Normal, self.searchString)
        if ok:
            self.searchString = text
            self.doFind(self.searchString)

    def onFindAgain(self):
        self.doFind(self.searchString)
