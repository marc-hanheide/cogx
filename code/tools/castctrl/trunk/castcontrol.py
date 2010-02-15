#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author: Marko Mahnič
# Created: June 2009

import os, sys, time
import re
import tempfile
from PyQt4 import QtCore, QtGui

from core import procman, options, messages, confbuilder, network
from core import castagentsrv, remoteproc
from core import legacy
from qtui import uimainwindow, uiresources
import processtree

LOGGER = messages.CInternalLogger()
procman.LOGGER = LOGGER
castagentsrv.LOGGER = LOGGER

class CLogDisplayer:
    def __init__(self, qtext):
        self.log = messages.CLogMerger()
        self.qtext = qtext
        doc = qtext.document()
        doc.setMaximumBlockCount(500)
        self.showFlush = False
        self.showWarning = True
        self.showError = True
        self.reError = re.compile(r"\b(error)\b", re.IGNORECASE)
        self.reWarning = re.compile(r"\b(warning)\b", re.IGNORECASE)

    def _markWords(self, text):
        text = self.reError.sub(r'<span style="background-color: yellow;"> \1 </span>', text)
        text = self.reWarning.sub(r'<span style="background-color: #ddddff;"> \1 </span>', text)
        return text

    def pullLogs(self):
        mods = False
        self.log.merge()
        if len(self.log.messages) < 1: pass
        else:
            pntr = messages.CAnsiPainter()
            msgs = self.log.getNewMessages(100)
            for m in msgs:
                if m.msgtype == messages.CMessage.CASTLOG:
                    self.qtext.append(pntr.paint(m.getText()))
                else:
                    text = m.getText()
                    co = None
                    if m.msgtype == messages.CMessage.WARNING:
                        if not self.showWarning: continue
                        co = "blue"
                    elif m.msgtype == messages.CMessage.ERROR:
                        if not self.showError: continue
                        text = self._markWords(text)
                        co = "red"
                    elif m.msgtype == messages.CMessage.FLUSHMSG:
                        if not self.showFlush: continue
                        text = self._markWords(text)
                        co = "grey"
                    if co == None: self.qtext.append(text)
                    else: self.qtext.append("<font color=%s>%s</font> " % (co, text))
            mods = True
        return mods

    def clearOutput(self):
        self.qtext.document().clear()

    def rereadLogs(self):
        for src in self.log.sources: src.restart()

    def saveLogs(self, stream): # TODO: save all available messages from currently registered sourcess
        # maybe: restart, pullRawLogs-->stream, clearOutput, restart
        pass

class CCastControlWnd(QtGui.QMainWindow):
    def __init__(self):
        QtGui.QMainWindow.__init__(self)
        self.ui = uimainwindow.Ui_MainWindow()
        self.ui.setupUi(self)

        self.fnconf = "castcontrol.conf"
        self.fnhist = "castcontrol.hist"
        self._options = options.CCastOptions()
        self._options.loadConfig(self.fnconf)
        if os.path.exists(self.fnhist): self._options.loadHistory(self.fnhist)
        else: self._options.loadHistory(self.fnconf) # old place for history
        self._options.configEnvironment()
        self._userOptions = options.CUserOptions()
        self._userOptions.loadConfig(self.fnconf) # TODO: user options should be in ~/.cast/...
        self.ui.txtLocalHost.setText(self._options.getOption("localhost"))
        self._manager = procman.CProcessManager()
        self._remoteHosts = []
        self._pumpRemoteMessages = True

        self.mainLog  = CLogDisplayer(self.ui.mainLogfileTxt)
        self.mainLog.log.addSource(LOGGER)

        self.buildLog  = CLogDisplayer(self.ui.buildLogfileTxt)
        self.buildLog.showFlush = True

        self._processModel = processtree.CProcessTreeModel()
        self.ui.processTree.setModel(self._processModel)

        self._initContent()
        self._initLocalProcesses()
        for proc in self._manager.proclist:
            if proc != self.procBuild: self.mainLog.log.addSource(proc)

        # Auxiliary components
        self.tmStatus = QtCore.QTimer()
        QtCore.QObject.connect(self.tmStatus, QtCore.SIGNAL("timeout()"), self.statusUpdate)
        self.tmStatus.start(632)

        # Event connections
        self.connect(self.ui.actQuit, QtCore.SIGNAL("triggered()"), self.close)
        self.connect(self.ui.actOpenClientConfig, QtCore.SIGNAL("triggered()"), self.onBrowseClientConfig)
        self.connect(self.ui.actOpenPlayerConfig, QtCore.SIGNAL("triggered()"), self.onBrowsePlayerConfig)
        self.connect(self.ui.actOpenHostConfig, QtCore.SIGNAL("triggered()"), self.onBrowseHostConfig)
        self.connect(self.ui.actShowEnv, QtCore.SIGNAL("triggered()"), self.onShowEnvironment)
        self.connect(self.ui.actStartTerminal, QtCore.SIGNAL("triggered()"), self.onStartTerminal)
        self.connect(self.ui.clientConfigCmbx, QtCore.SIGNAL("currentIndexChanged(int)"), self.onClientConfigChanged)
        self.connect(self.ui.playerConfigCmbx, QtCore.SIGNAL("currentIndexChanged(int)"), self.onPlayerConfigChanged)
        self.connect(self.ui.hostConfigCmbx, QtCore.SIGNAL("currentIndexChanged(int)"), self.onHostConfigChanged)
        self.connect(self.ui.actCtxShowBuildError, QtCore.SIGNAL("triggered()"), self.onEditBuildError)

        # Context menu actions for QTextEdit
        self.ui.buildLogfileTxt.contextActions.append(self.ui.actCtxShowBuildError)

        pic = uiresources.createPixmap(uiresources.icon_cogx)
        self.setWindowIcon(QtGui.QIcon(pic))
        LOGGER.log("CAST Control initialized")

    def _initContent(self):
        for fn in self._options.mruCfgCast:
            if fn.strip() == "": continue
            self.ui.clientConfigCmbx.addItem(self.makeConfigFileDisplay(fn), QtCore.QVariant(fn))
        for fn in self._options.mruCfgPlayer:
            if fn.strip() == "": continue
            self.ui.playerConfigCmbx.addItem(self.makeConfigFileDisplay(fn), QtCore.QVariant(fn))
        for fn in self._options.mruCfgHosts:
            if fn.strip() == "": continue
            self.ui.hostConfigCmbx.addItem(self.makeConfigFileDisplay(fn), QtCore.QVariant(fn))

    def makeConfigFileDisplay(self, fn):
        fn = "%s" % fn
        return "%s   @ %s" % (os.path.basename(fn), os.path.dirname(fn))

    # If the file is under the COGX_ROOT directory, make it relative
    def makeConfigFileRelPath(self, fn):
        wd = options.xe("${COGX_ROOT}")
        if wd.strip() == "": return fn
        rp = legacy.os_path_relpath("%s" % fn, wd)
        if rp.startswith(".."): return fn
        return rp

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

    @property
    def _hostConfig(self):
        cmb = self.ui.hostConfigCmbx
        i = cmb.currentIndex()
        fn = cmb.itemData(i)
        return fn.toString()

    @property
    def _localhost(self):
        lhost = "%s" % self.ui.txtLocalHost.text()
        lhost = lhost.strip()
        return lhost

    def _initLocalProcesses(self):
        self._manager.addProcess(procman.CProcess("server-java", options.xe("${CMD_JAVA_SERVER}")))
        self._manager.addProcess(procman.CProcess("server-cpp", options.xe("${CMD_CPP_SERVER}")))
        self._manager.addProcess(procman.CProcess("server-python", options.xe("${CMD_PYTHON_SERVER}")))
        self._manager.addProcess(procman.CProcess("client", options.xe("${CMD_CAST_CLIENT}")))
        self._manager.addProcess(procman.CProcess("player", options.xe("${CMD_PLAYER}")))
        self._manager.addProcess(procman.CProcess("peekabot", options.xe("${CMD_PEEKABOT}")))
        self.procBuild = procman.CProcess("BUILD", 'make [target]', workdir=options.xe("${COGX_BUILD_DIR}"))
        self.procBuild.allowTerminate = True
        self._manager.addProcess(self.procBuild)
        self._processModel.rootItem.addHost(self._manager)
        self.ui.processTree.expandAll()

    def statusUpdate(self):
        self.mainLog.showFlush = self.ui.ckShowFlushMsgs.isChecked()
        self.mainLog.pullLogs()
        self.buildLog.pullLogs()
        for rpm in self._remoteHosts: #TODO: read in background, at most 1 per second
            rpm.updateProcessList()
            if self._pumpRemoteMessages: rpm.pumpRemoteMessages()

    def closeEvent(self, event):
        self._manager.stopReaderThread()
        self._manager.stopAll()
        time.sleep(0.2)
        def getitems(cmbx, count=30):
            mx = cmbx.count()
            if mx > count: mx = count
            lst = []
            for i in xrange(mx):
                fn = cmbx.itemData(i).toString()
                lst.append("%s" % fn)
            return lst

        try:
            self._options.mruCfgCast = getitems(self.ui.clientConfigCmbx)
            self._options.mruCfgPlayer = getitems(self.ui.playerConfigCmbx)
            self._options.mruCfgHosts = getitems(self.ui.hostConfigCmbx)
            self._options.saveHistory(open(self.fnhist, 'w'))
            if not os.path.exists(self.fnconf):
                self._options.saveConfig(open(self.fnconf, 'w'))
        except Exception, e:
            print "Failed to save configuration"
            print e

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
            script.append(options.xe(stm))
        for i,cmd in enumerate(script):
            # procman.runCommand(cmd, name="cleanup-cmd-%d" % (i+1))
            procman.xrun_wait(cmd)

    def startServers(self):
        if self.ui.ckCleanupScript.isChecked():
            self.runCleanupScript()
        srvs = self.getServers(self._manager)
        for p in srvs: p.start()
        
        # XXX Processes could be started for each host separately;  (id=remotestart)
        # Each process on each host could be started separately;
        # ATM we start all processes on all hosts, so the UI can remain unchanged.
        for h in self._remoteHosts:
            for p in h.proclist: p.start()
        self.ui.processTree.expandAll()

    def stopServers(self):
        srvs = self.getServers(self._manager)
        for p in srvs: p.stop()

        for h in self._remoteHosts: # See <URL:#remotestart>
            for p in h.proclist: p.stop()
        self.ui.processTree.expandAll()

    # Somehow we get 2 events for a button click ... filter one out
    def on_btServerStart_clicked(self, valid=True):
        if not valid: return
        self.startServers()

    def on_btServerStop_clicked(self, valid=True):
        if not valid: return
        self.stopServers()

    # build config file from rules in hostconfig
    def buildConfigFile(self):
        if self._clientConfig == None: clientConfig = ""
        else: clientConfig = "%s" % self._clientConfig
        clientConfig = clientConfig.strip()
        if self._hostConfig == None: hostConfig = ""
        else: hostConfig = "%s" % self._hostConfig
        hostConfig = hostConfig.strip()
        lhost = self._localhost
        if clientConfig == "": return clientConfig
        if lhost == "" and hostConfig == "": return clientConfig
        LOGGER.log("LOCALHOST: %s" % lhost)

        self._tmpCastFile = tempfile.NamedTemporaryFile(suffix=".cast")
        cfg = confbuilder.CCastConfig()
        cfg.clearRules()
        if lhost != "": cfg.setLocalhost(lhost)
        if hostConfig != "": cfg.addRules(open(hostConfig, "r").readlines())
        cfg.prepareConfig(clientConfig, self._tmpCastFile)
        self._tmpCastFile.flush()
        # don't self._tmpCastFile.close()
        return self._tmpCastFile.name


    def getConfiguredHosts(self):
        if self._hostConfig == None: hostConfig = ""
        else: hostConfig = "%s" % self._hostConfig
        hostConfig = hostConfig.strip()
        lhost = self._localhost
        cfg = confbuilder.CCastConfig()
        cfg.clearRules()
        if lhost != "": cfg.setLocalhost(lhost)
        if hostConfig == "" or not os.path.exists(hostConfig):
            msg = "Host configuration file (.hconf) not defined or it doesn't exist."
            LOGGER.warn(msg)
            raise UserWarning("HCONF: " + msg)
            return []
            
        cfg.addRules(open(hostConfig, "r").readlines())

        hosts = []
        for hid,haddr in cfg.hosts.iteritems():
            if hid[:4] == "127.": continue
            if haddr == "localhost" or haddr[:4] == "127.": continue
            # if haddr == lhost: continue # XXX Comment this line when testing and an agent is on localhost
            if not haddr in hosts: hosts.append(haddr)

        return hosts


    def discoverCastAgents(self):
        hosts = self.getConfiguredHosts()
        LOGGER.log("Hosts: %s" % (hosts))
        port = castagentsrv.SLAVE_PORT # TODO: user setting, maybe per-host setting
        working = castagentsrv.discoverRemoteHosts(hosts, port)
        LOGGER.log("Cast agents: %s" % (working))

        return working


    def on_btClientStart_clicked(self, valid=True):
        if not valid: return
        p = self._manager.getProcess("client")
        if p != None:
            self.ui.tabWidget.setCurrentWidget(self.ui.tabLogs)
            try:
                confname = self.buildConfigFile()
                if confname != None and confname != "":
                    p.start( params = { "CAST_CONFIG": confname } )
            except Exception as e:
                LOGGER.error("%s" % e)
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

    def _checkBuidDir(self):
        workdir=options.xe("${COGX_BUILD_DIR}")
        if not os.path.exists(workdir):
            dlg = QtGui.QErrorMessage(self)
            dlg.setModal(True)
            dlg.showMessage("The build directory doesn't exist. Run 'Config...'")
            return False
        return True

    def on_btBuild_clicked(self, valid=True):
        if not valid: return
        if not self._checkBuidDir(): return
        p = self._manager.getProcess("BUILD")
        if p != None:
            self.buildLog.clearOutput()
            if not self.buildLog.log.hasSource(p): self.buildLog.log.addSource(p)
            p.start(params={"target": ""})
            # p.start()

    def on_btBuildInstall_clicked(self, valid=True):
        if not valid: return
        if not self._checkBuidDir(): return
        p = self._manager.getProcess("BUILD")
        if p != None:
            self.buildLog.clearOutput()
            if not self.buildLog.log.hasSource(p): self.buildLog.log.addSource(p)
            p.start(params={"target": "install"})

    def _checkMakeCache(self, listsDir, cacheFile):
        bdir = os.path.dirname(cacheFile)
        if not os.path.exists(bdir): os.makedirs(bdir)
        if not os.path.exists(cacheFile): return True

        # build directory in: second line of cache (comment)
        # source directory in: CMAKE_HOME_DIRECTORY:INTERNAL=
        src = ""; dst = ""
        cache = open(cacheFile).readlines()
        line = cache[1]
        if line.startswith("# For build in directory:"):
            pos = line.find(":")
            dst = line[pos+1:].strip()
        for line in cache:
            if line.startswith("CMAKE_HOME_DIRECTORY:INTERNAL="):
                pos = line.find("=")
                src = line[pos+1:].strip()
                break
        if src != "" and dst != "":
            if src != listsDir or dst != bdir:
                QM = QtGui.QMessageBox
                rv = QtGui.QMessageBox.warning(self, "CAST Control",
                    "CMakeCache.txt points to wrong directories.\n"
                    + "Shall I fix that?", QM.Yes | QM.No)
                if rv == QM.Yes:
                    rv = QtGui.QMessageBox.information(
                        self, "CAST Control", "TODO: Maybe Someday I will be able to fix CMakeCache...")
                    # TODO: Change directories on YES
                pass
        return True

    def on_btCmakeGui_clicked(self, valid=True):
        if not valid: return
        root = options.xe("${COGX_ROOT}")
        bdir = options.xe("${COGX_BUILD_DIR}")
        bcmc = os.path.join(bdir, "CMakeCache.txt")
        if not self._checkMakeCache(root, bcmc): return
        cmd = 'cmake-gui %s' % root
        procman.xrun_wait(cmd, bdir)

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
        if self._pumpRemoteMessages:
            for h in self._remoteHosts:
                src = remoteproc.CRemoteMessageSource(h)
                # XXX Bad interface in CLogMerger (addSource accepts a process instead of CMessageSource)
                self.mainLog.log.removeSource(h)
                self.mainLog.log.sources.append(src)

    def on_btClearMainLog_clicked(self, valid=True):
        if not valid: return
        self.mainLog.clearOutput()

    def on_ckShowFlushMsgs_stateChanged(self, value):
        self.mainLog.clearOutput()
        self.mainLog.rereadLogs()

    def editFile(self, filename, line=None):
        cmd = self._userOptions.textEditCmd
        mo = re.search("%l(\[([^\]]+)\])?", cmd)
        if mo != None:
            if line == None: lexpr = ""
            else: lexpr = "%s%d" % (mo.group(2), line)
            cmd = cmd[:mo.start()] + lexpr + cmd[mo.end():]
        procman.xrun(cmd % filename)

    def on_btEditClientConfig_clicked(self, valid=True):
        if not valid: return
        self.editFile(self._clientConfig)

    def on_btEditPlayerConfig_clicked(self, valid=True):
        if not valid: return
        self.editFile(self._playerConfig)

    def on_btEditHostConfig_clicked(self, valid=True):
        if not valid: return
        self.editFile(self._hostConfig)

    def on_btDetectLocalHost_clicked(self, valid=True):
        if not valid: return
        ip = network.get_ip_address()
        if ip != None:
            self.ui.txtLocalHost.setText(ip.strip())
            self._options.setOption("localhost", ip.strip())

    def on_btDiscoverRemote_clicked(self, valid=True):
        if not valid: return
        try:
            agents = self.discoverCastAgents()
        except UserWarning as uw:
            rv = QtGui.QMessageBox.information(self, "Discover Agents", "%s" % uw)
            return

        for rpm in self._remoteHosts:
            self._processModel.rootItem.removeHost(rpm)
            self.mainLog.log.removeSource(rpm)
        self._remoteHosts = []

        for host in agents:
            port = castagentsrv.SLAVE_PORT # TODO: user setting, maybe per-host setting
            rpm = remoteproc.CRemoteProcessManager(host, host, port)
            rpm.updateProcessList()
            self._remoteHosts.append(rpm)
            self._processModel.rootItem.addHost(rpm)
            if self._pumpRemoteMessages:
                src = remoteproc.CRemoteMessageSource(rpm)
                self.mainLog.log.sources.append(src)

        self.ui.processTree.expandAll()


    def onShowEnvironment(self):
        cmd = "bash -c env"
        # procman.runCommand(cmd, name="ENV")
        procman.xrun_wait(cmd)

    def onStartTerminal(self):
        root = options.xe("${COGX_ROOT}")
        cmd = self._userOptions.terminalCmd
        if cmd.find("%s") > 0: cmd = cmd % root
        procman.xrun(cmd)

    def onEditBuildError(self):
        tcur = self.ui.buildLogfileTxt.textCursor()
        line = "%s" % tcur.block().text().trimmed()
        mo = re.search("\s(/[^:]+)" + ":(\d+)" + "(:(\d+))?", line)
        if mo != None:
            self.editFile(mo.group(1), int(mo.group(2)))

    def onBrowseClientConfig(self):
        qfd = QtGui.QFileDialog
        fn = qfd.getOpenFileName(
            self, self.ui.actOpenClientConfig.text(),
            "", "CAST Config (*.cast)")
        if fn != None and len(fn) > 1:
            fn = self.makeConfigFileRelPath(fn)
            self._ComboBox_AddMru(self.ui.clientConfigCmbx,
                    self.makeConfigFileDisplay(fn), QtCore.QVariant(fn))

    def onClientConfigChanged(self, index):
        if index < 1: return
        fn = self._clientConfig
        self._ComboBox_SelectMru(self.ui.clientConfigCmbx, index,
                self.makeConfigFileDisplay(fn), QtCore.QVariant(fn))

    def onBrowsePlayerConfig(self):
        qfd = QtGui.QFileDialog
        fn = qfd.getOpenFileName(
            self, self.ui.actOpenPlayerConfig.text(),
            "", "Player Config (*.cfg)")
        if fn != None and len(fn) > 1:
            fn = self.makeConfigFileRelPath(fn)
            self._ComboBox_AddMru(self.ui.playerConfigCmbx,
                    self.makeConfigFileDisplay(fn), QtCore.QVariant(fn))

    def onPlayerConfigChanged(self, index):
        if index < 1: return
        fn = self._playerConfig
        self._ComboBox_SelectMru(self.ui.playerConfigCmbx, index,
                self.makeConfigFileDisplay(fn), QtCore.QVariant(fn))

    def onBrowseHostConfig(self):
        qfd = QtGui.QFileDialog
        fn = qfd.getOpenFileName(
            self, self.ui.actOpenHostConfig.text(),
            "", "Component Host Config (*.hconf)")
        if fn != None and len(fn) > 1:
            fn = self.makeConfigFileRelPath(fn)
            self._ComboBox_AddMru(self.ui.hostConfigCmbx,
                    self.makeConfigFileDisplay(fn), QtCore.QVariant(fn))

    def onHostConfigChanged(self, index):
        if index < 1: return
        fn = self._hostConfig
        self._ComboBox_SelectMru(self.ui.hostConfigCmbx, index,
                self.makeConfigFileDisplay(fn), QtCore.QVariant(fn))

    def _ComboBox_AddMru(self, uiCmbx, title, varData):
        uiCmbx.blockSignals(True)
        uiCmbx.insertItem(0, title, varData)
        uiCmbx.setCurrentIndex(0)
        uiCmbx.blockSignals(False)

    # Move an item that was picked (index) to the front of MRU list (index = 0)
    def _ComboBox_SelectMru(self, uiCmbx, index, title, varData):
        uiCmbx.blockSignals(True)
        uiCmbx.removeItem(index)
        uiCmbx.insertItem(0, title, varData)
        uiCmbx.setCurrentIndex(0)
        uiCmbx.blockSignals(False)


def guiMain():
    app = QtGui.QApplication(sys.argv)
    myapp = CCastControlWnd()
    myapp.show()
    sys.exit(app.exec_())
    pass

if __name__ == "__main__": guiMain()

