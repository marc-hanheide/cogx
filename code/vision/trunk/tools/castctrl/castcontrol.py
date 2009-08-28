#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author: Marko Mahnič
# Created: June 2009

import os, sys
from PyQt4 import QtCore, QtGui

from core import procman, options, messages
from qtui import uimainwindow
import processtree

LOGGER = messages.CInternalLogger()
procman.LOGGER = LOGGER

def xrun(cmdline):
    if type(cmdline) == type(""): cmds = cmdline.split()
    else: cmds = cmdline # assume list
    cmd = cmds[0]
    return os.spawnvp(os.P_NOWAIT, cmd, cmds)

class CLogDisplayer:
    def __init__(self, qtext):
        self.log = messages.CLogMerger()
        self.qtext = qtext
        doc = qtext.document()
        doc.setMaximumBlockCount(500)

    def pullLogs(self):
        mods = False
        self.log.merge()
        if len(self.log.messages) < 1: pass
        else:
            pntr = messages.CAnsiPainter()
            msgs = [m for m in self.log.messages]
            self.log.clearBuffer()
            for m in msgs:
                if m.msgtype == messages.CMessage.CASTLOG:
                    self.qtext.append(pntr.paint(m.getText()))
                else:
                    co = None
                    if m.msgtype == messages.CMessage.WARNING: co = "blue"
                    elif m.msgtype == messages.CMessage.ERROR: co = "red"
                    if co == None: self.qtext.append(m.getText())
                    else: self.qtext.append("<font color=%s>%s</font> " % (co, m.getText().rstrip()))
            mods = True
        return mods

    def clearOutput(self):
        self.qtext.document().clear()

class CCastControlWnd(QtGui.QMainWindow):
    def __init__(self):
        QtGui.QMainWindow.__init__(self)
        self.ui = uimainwindow.Ui_MainWindow()
        self.ui.setupUi(self)

        self._options = options.CCastOptions()
        self._options.loadConfig("castcontrol.conf")
        self._options.configEnvironment()
        self._userOptions = options.CUserOptions()
        self._manager = procman.CProcessManager()

        self.mainLog  = CLogDisplayer(self.ui.mainLogfileTxt)
        self.mainLog.log.addSource(LOGGER)

        self.buildLog  = CLogDisplayer(self.ui.buildLogfileTxt)
        # self.buildLog.log.addSource(LOGGER)

        self.mruCfgCast = []
        self.mruCfgPlayer = []

        self._processModel = processtree.CProcessTreeModel()
        self.ui.processTree.setModel(self._processModel)

        self._initContent()
        self._initLocalProcesses()

        # Auxiliary components
        self.tmStatus = QtCore.QTimer()
        QtCore.QObject.connect(self.tmStatus, QtCore.SIGNAL("timeout()"), self.statusUpdate)
        self.tmStatus.start(100)
        LOGGER.log("CAST Control initialized")

        # Event connections
        self.connect(self.ui.actQuit, QtCore.SIGNAL("triggered()"), self.close)

    def _initContent(self):
        for i in self._options.mruCfgCast:
            self.ui.clientConfigCmbx.addItem(i)
        for i in self._options.mruCfgPlayer:
            self.ui.playerConfigCmbx.addItem(i)

    def _initLocalProcesses(self):
        self._manager.addProcess(procman.CProcess("server-java", options.xe("${CMD_JAVA_SERVER}")))
        self._manager.addProcess(procman.CProcess("server-cpp", options.xe("${CMD_CPP_SERVER}")))
        self._manager.addProcess(procman.CProcess("server-python", options.xe("${CMD_PYTHON_SERVER}")))
        self._manager.addProcess(procman.CProcess("client", options.xe("${CMD_CAST_CLIENT}")))
        self._manager.addProcess(procman.CProcess("player", options.xe("${CMD_PLAYER}")))
        self._manager.addProcess(procman.CProcess("peekabot", options.xe("${CMD_PEEKABOT}")))
        self.procBuild = procman.CProcess("BUILD", 'make [cmd]', workdir=options.xe("${SA_BUILD_DIR}"))
        self.procBuild.allowTerminate = True
        self._manager.addProcess(self.procBuild)
        self._processModel.rootItem.addHost(self._manager)
        self.ui.processTree.expandAll()

    def statusUpdate(self):
        rv = self._manager.checkProcesses()
        self._manager.communicate()
        self.mainLog.pullLogs()
        self.buildLog.pullLogs()
        # self.updateUi()

    def closeEvent(self, event):
        self._manager.stopAll()

    def getServers(self, manager):
        srvs = []
        p = manager.getProcess("server-java")
        if p != None: srvs.append(p)
        p = manager.getProcess("server-cpp")
        if p != None: srvs.append(p)
        p = manager.getProcess("server-python")
        if p != None: srvs.append(p)
        return srvs

    def runCleanupScript(self):
        script = []
        for stm in self._options.cleanupScript:
            stm = stm.split('#')[0]
            stm = stm.strip()
            if len(stm) < 1: continue
            script.append(stm)
        for i,cmd in enumerate(script):
            procman.runCommand(cmd, name="cleanup-cmd-%d" % (i+1))

    # Somehow we get 2 events for a button click ... filter one out
    def on_btServerStart_clicked(self, valid=True):
        if not valid: return
        if self.ui.ckCleanupScript.isChecked():
            self.runCleanupScript()
        srvs = self.getServers(self._manager)
        for p in srvs: p.start()

    def on_btServerStop_clicked(self, valid=True):
        if not valid: return
        srvs = self.getServers(self._manager)
        for p in srvs: p.stop()

    def on_btClientStart_clicked(self, valid=True):
        if not valid: return
        p = self._manager.getProcess("client")
        if p != None: p.start( params = { "CAST_CONFIG": self.ui.clientConfigCmbx.currentText() } )
        # if p != None: p.start( params = { "CAST_CONFIG": self._options.mruCfgCast[0] } )

    def on_btClientStop_clicked(self, valid=True):
        if not valid: return
        p = self._manager.getProcess("client")
        if p != None: p.stop()

    def on_btPlayerStart_clicked(self, valid=True):
        if not valid: return
        p = self._manager.getProcess("player")
        if p != None: p.start( params = { "PLAYER_CONFIG": self.ui.playerConfigCmbx.currentText() } )
        if self.ui.ckPeekabot.isChecked():
            p = self._manager.getProcess("peekabot")
            if p != None: p.start()

    def on_btPlayerStop_clicked(self, valid=True):
        if not valid: return
        p = self._manager.getProcess("player")
        if p != None: p.stop()
        p = self._manager.getProcess("peekabot")
        if p != None: p.stop()

    def on_btBuild_clicked(self, valid=True):
        if not valid: return
        p = self._manager.getProcess("BUILD")
        if p != None:
            self.buildLog.clearOutput()
            if not self.buildLog.log.hasSource(p): self.buildLog.log.addSource(p)
            p.start(params={"cmd": ""})
            # p.start()

    def on_btBuildInstall_clicked(self, valid=True):
        if not valid: return
        p = self._manager.getProcess("BUILD")
        if p != None:
            self.buildLog.clearOutput()
            if not self.buildLog.log.hasSource(p): self.buildLog.log.addSource(p)
            p.start(params={"cmd": "install"})

    def on_btLogViewControl_clicked(self, valid=True):
        if not valid: return
        self.mainLog.log.removeAllSources()
        self.mainLog.clearOutput()
        self.mainLog.log.addSource(LOGGER)

    def on_btLogViewAll_clicked(self, valid=True):
        if not valid: return
        self.mainLog.log.removeAllSources()
        self.mainLog.clearOutput()
        self.mainLog.log.addSource(LOGGER)
        for proc in self._manager.proclist:
            if proc != self.procBuild: self.mainLog.log.addSource(proc)

    def editFile(self, fn):
        shell = "/bin/sh"
        cmd = self._userOptions.textEditCmd
        xrun([shell, "-c", cmd % fn, " &"])

    def on_btEditClientConfig_clicked(self, valid=True):
        if not valid: return
        self.editFile(self.ui.clientConfigCmbx.currentText())

    def on_btEditPlayerConfig_clicked(self, valid=True):
        if not valid: return
        self.editFile(self.ui.playerConfigCmbx.currentText())


def guiMain():
    app = QtGui.QApplication(sys.argv)
    # myapp = ManagerWindow()
    myapp = CCastControlWnd()
    myapp.show()
    sys.exit(app.exec_())
    pass

guiMain()
