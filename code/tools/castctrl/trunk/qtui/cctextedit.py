#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim

from PyQt4 import QtCore, QtGui

class CCastTextEdit(QtGui.QTextEdit):
    def __init__(self, parent):
        QtGui.QTextEdit.__init__(self, parent)
        self.contextActions = []

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
    pass
