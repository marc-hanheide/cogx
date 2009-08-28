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
        self.connect(self.ui.actOpenClientConfig, QtCore.SIGNAL("triggered()"), self.onBrowseClientConfig)
        self.connect(self.ui.actOpenPlayerConfig, QtCore.SIGNAL("triggered()"), self.onBrowsePlayerConfig)
        self.connect(self.ui.clientConfigCmbx, QtCore.SIGNAL("currentIndexChanged(int)"), self.onClientConfigChanged)

    def _initContent(self):
        for fn in self._options.mruCfgCast:
            if fn.strip() == "": continue
            self.ui.clientConfigCmbx.addItem(self.makeConfigFileDisplay(fn), QtCore.QVariant(fn))
        for fn in self._options.mruCfgPlayer:
            if fn.strip() == "": continue
            self.ui.playerConfigCmbx.addItem(self.makeConfigFileDisplay(fn), QtCore.QVariant(fn))

    def makeConfigFileDisplay(self, fn):
        fn = "%s" % fn
        return "%s \t- %s" % (os.path.basename(fn), os.path.dirname(fn))

    @property
    def _clientConfig(self):
        cmb = self.ui.clientConfigCmbx
        i = cmb.currentIndex()
        fn = cmb.itemData(i)
        return fn.toString()

    @property
    def _playerConfig(self):
        cmb = self.ui.playerConfigCmbx
        i = cmb.currentIndex()
        fn = cmb.itemData(i)
        return fn.toString()

    def _initLocalProcesses(self):
        self._manager.addProcess(procman.CProcess("server-java", options.xe("${CMD_JAVA_SERVER}")))
        self._manager.addProcess(procman.CProcess("server-cpp", options.xe("${CMD_CPP_SERVER}")))
        self._manager.addProcess(procman.CProcess("server-python", options.xe("${CMD_PYTHON_SERVER}")))
        self._manager.addProcess(procman.CProcess("client", options.xe("${CMD_CAST_CLIENT}")))
        self._manager.addProcess(procman.CProcess("player", options.xe("${CMD_PLAYER}")))
        self._manager.addProcess(procman.CProcess("peekabot", options.xe("${CMD_PEEKABOT}")))
        self.procBuild = procman.CProcess("BUILD", 'make [cmd]', workdir=options.xe("${COGX_BUILD_DIR}"))
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
        if p != None: p.start( params = { "CAST_CONFIG": self._clientConfig } )
        # if p != None: p.start( params = { "CAST_CONFIG": self._options.mruCfgCast[0] } )

    def on_btClientStop_clicked(self, valid=True):
        if not valid: return
        p = self._manager.getProcess("client")
        if p != None: p.stop()

    def on_btPlayerStart_clicked(self, valid=True):
        if not valid: return
        p = self._manager.getProcess("player")
        if p != None: p.start( params = { "PLAYER_CONFIG": self._playerConfig } )
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
        self.editFile(self._clientConfig)

    def on_btEditPlayerConfig_clicked(self, valid=True):
        if not valid: return
        self.editFile(self._playerConfig)

    def onBrowseClientConfig(self):
        qfd = QtGui.QFileDialog
        fn = qfd.getOpenFileName(
            self, self.ui.actOpenClientConfig.text(),
            "", "CAST Config (*.cast)")
        if fn != None and len(fn) > 1:
            self.ui.clientConfigCmbx.blockSignals(True)
            self.ui.clientConfigCmbx.insertItem(0, self.makeConfigFileDisplay(fn), QtCore.QVariant(fn))
            self.ui.clientConfigCmbx.setCurrentIndex(0)
            self.ui.clientConfigCmbx.blockSignals(False)

    def onBrowsePlayerConfig(self):
        qfd = QtGui.QFileDialog
        fn = qfd.getOpenFileName(
            self, self.ui.actOpenPlayerConfig.text(),
            "", "Player Config (*.cfg)")
        if fn != None and len(fn) > 1:
            self.ui.clientConfigCmbx.blockSignals(True)
            self.ui.playerConfigCmbx.insertItem(0, self.makeConfigFileDisplay(fn), QtCore.QVariant(fn))
            self.ui.playerConfigCmbx.setCurrentIndex(0)
            self.ui.clientConfigCmbx.blockSignals(False)

    def onClientConfigChanged(self, index):
        if index < 1: return
        fn = self._clientConfig
        self.ui.clientConfigCmbx.blockSignals(True)
        self.ui.clientConfigCmbx.removeItem(index)
        self.ui.clientConfigCmbx.insertItem(0, self.makeConfigFileDisplay(fn), QtCore.QVariant(fn))
        self.ui.clientConfigCmbx.setCurrentIndex(0)
        self.ui.clientConfigCmbx.blockSignals(False)

def guiMain():
    app = QtGui.QApplication(sys.argv)
    myapp = CCastControlWnd()
    myapp.show()
    sys.exit(app.exec_())
    pass

guiMain()
