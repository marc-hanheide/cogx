#!/usr/bin/python
# vim: set sw=4 ts=8 sts=4 et :vim
#

import os, sys
#from PyQt4 import QtCore, QtGui
from PyQt4 import Qt
from PyQt4.QtCore import *
from PyQt4.QtGui import *
from PyQt4 import uic
from PyQt4.QtScript import *
from PyQt4.QtScriptTools import *

dlgOwner = """
function CDialogOwner()
{
}
CDialogOwner.prototype.setValue = function(name, value)
{
}
CDialogOwner.prototype.call = function(command, params)
{
}
var dialogOwner = new CDialogOwner();
"""
#class CCastOwnerProxy(QObject):
#    def setValue(self, name, value):
#        print "Setting %s='%s'" % (name, value)

def run(fnameUi, fnameJs, construct):
    app = QApplication(sys.argv)
    engine = QScriptEngine()

    debugger = QScriptEngineDebugger()
    debugger.attachTo(engine)
    debugWindow = debugger.standardWindow()
    debugWindow.resize(1024, 600)

    codeJs = open(fnameJs).read()
    engine.evaluate(codeJs)
    engine.evaluate(dlgOwner)

    #ownerProxy = engine.newQObject(CCastOwnerProxy(), QScriptEngine.ScriptOwnership)
    #glob = engine.globalObject()
    #glob.setProperty("dialogOwner", ownerProxy)

    widget = uic.loadUi(fnameUi)

    ctor = engine.evaluate(construct)
    scriptUi = engine.newQObject(widget, QScriptEngine.ScriptOwnership)
    calc = ctor.construct([scriptUi])

    widget.show()
    return app.exec_()




run("ptucontroller.ui", "ptucontroller.js", "PtuController")
# run("calculator.ui", "calculator.js", "Calculator")
