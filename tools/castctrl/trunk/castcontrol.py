#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author: Marko Mahnič
# Created: June 2009

import os, sys, time, signal
import re
import tempfile
import threading
import ConfigParser
from PyQt4 import QtCore, QtGui

from core import procman, options, messages, logger, confbuilder, network
from core import castagentsrv, remoteproc
from core import legacy
from core import log4util
from core import rasync
from qtui import uimainwindow
from selectcomponentdlg import CSelectComponentsDlg
from textedit import CTextEditor
import processtree

import pconfig
from pconfig.configwidget import CConfigWidget
from pconfig.manager import CServerManager

LOGGER = logger.get()
NOFILE_FILENAME = "<none>"

class CLambdaThread(threading.Thread):
    def __init__(self, data, func):
        threading.Thread.__init__(self)
        self.data = data
        self.func = func
    def run(self):
        self.func(self.data)

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

    def setMaxBlockCount(self, count):
        doc = self.qtext.document()
        doc.setMaximumBlockCount(count)

    def pullLogs(self):
        mods = False
        self.log.merge()
        msgs = self.log.getNewMessages(200)
        if len(msgs) > 0:
            pntr = messages.CAnsiPainter()
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
                    # text = m.source + ": " + text
                    if co == None: self.qtext.append(text)
                    else: self.qtext.append("<font color=%s>%s</font> " % (co, text))
            mods = True
        return mods

    def clearOutput(self):
        self.qtext.document().clear()

    def setFilter(self, fnFilter):
        self.clearOutput()
        self.log.setFilter(fnFilter, 500) # setMaximumBlockCount

    def rereadLogs(self):
        self.clearOutput()
        self.log.restart(500) # setMaximumBlockCount

    def saveLogs(self, stream): # TODO: save all available messages from currently registered sourcess
        # maybe: restart, pullRawLogs-->stream, clearOutput, restart
        pass

class CProcessGroup:
    def __init__(self, name):
        self.name = name
        self.processlist = []

    def addProcess(self, processInfo):
        self.processlist.append(processInfo)

class CCastControlWnd(QtGui.QMainWindow):
    def __init__(self):
        QtGui.QMainWindow.__init__(self)
        self._setup_ui()

        self.fnconf = "castcontrol.conf"
        self.fnhist = "castctrl.cache.ini"
        self._options = options.getCastOptions()
        self._options.loadConfig(self.fnconf)
        if os.path.exists(self.fnhist):
            cfg = ConfigParser.RawConfigParser()
            cfg.read(self.fnhist)
            self._options.loadHistory(cfg)
        self._options.configEnvironment()
        self._userOptions = options.getUserOptions()
        self._userOptions.loadConfig()
        self.componentFilter = []
        _localhost = self._options.getOption("localhost")
        self.configuredHosts = confbuilder.CHostMap(localhost=_localhost)
        self.log4jStartedOn = None

        # move option values to UI
        self.ui.txtLocalHost.setText(_localhost)

        self._manager = procman.CProcessManager()
        self._remoteHosts = []
        self._pumpRemoteMessages = True

        root = self._options.xe("${COGX_ROOT}")
        if len(root) > 64: root = "..." + root[-64:]
        self.setWindowTitle("CAST Control - " + root)

        self.mainLog  = CLogDisplayer(self.ui.mainLogfileTxt)
        self.mainLog.setMaxBlockCount(self._userOptions.maxMainLogLines)
        self.mainLog.log.addSource(LOGGER)

        self.buildLog  = CLogDisplayer(self.ui.buildLogfileTxt)
        self.buildLog.setMaxBlockCount(self._userOptions.maxBuildLogLines)
        self.buildLog.showFlush = True

        self._processModel = processtree.CProcessTreeModel()
        self.ui.processTree.setModel(self._processModel)

        self._initContent()
        self._initLocalProcesses()
        for proc in self._manager.proclist:
            if proc != self.procBuild: self.mainLog.log.addSource(proc)
        self._fillMessageFilterCombo()

        # Auxiliary components
        self.tmStatus = QtCore.QTimer()
        QtCore.QObject.connect(self.tmStatus, QtCore.SIGNAL("timeout()"), self.statusUpdate)
        self.tmStatus.start(632)

        self._connect_signals()
        LOGGER.log("CAST Control initialized")


    def _setup_ui(self):
        self.ui = uimainwindow.Ui_MainWindow()
        self.ui.setupUi(self)
        self.setWindowIcon(QtGui.QIcon(":icons/res/cogx_icon.png"))
        self.wAppConfig = CConfigWidget(self.ui.applicationTreeView)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        self.wAppConfig.setSizePolicy(sizePolicy)
        layout = QtGui.QVBoxLayout(self.ui.applicationTreeView)
        layout.addWidget(self.wAppConfig)

        # XXX keep the old interface, just in case, but hide it
        self.ui.tabWidget.removeTab(self.ui.tabWidget.indexOf(self.ui.tabOldInterface))


    def _connect_signals(self):
        # Event connections
        self.connect(self.ui.actQuit, QtCore.SIGNAL("triggered()"), self.close)
        self.connect(self.ui.actShowEnv, QtCore.SIGNAL("triggered()"), self.onShowEnvironment)

        # Config actions
        self.connect(self.ui.actOpenClientConfig, QtCore.SIGNAL("triggered()"), self.onBrowseClientConfig)
        self.connect(self.ui.actOpenHostConfig, QtCore.SIGNAL("triggered()"), self.onBrowseHostConfig)
        self.connect(self.ui.actStartTerminal, QtCore.SIGNAL("triggered()"), self.onStartTerminal)
        self.connect(self.ui.clientConfigCmbx, QtCore.SIGNAL("activated(int)"), self.onClientConfigChanged)
        self.connect(self.ui.hostConfigCmbx, QtCore.SIGNAL("activated(int)"), self.onHostConfigChanged)
        self.connect(self.ui.cbLog4jServerHost, QtCore.SIGNAL("activated(int)"), self.onLog4jHostCbItemActivated)

        # Process actions
        self.connect(self.ui.actStartCastServers, QtCore.SIGNAL("triggered()"), self.onStartCastServers)
        self.connect(self.ui.actStopCastServers, QtCore.SIGNAL("triggered()"), self.onStopCastServers)
        self.connect(self.ui.actStartCastClient, QtCore.SIGNAL("triggered()"), self.onStartCastClient)
        self.connect(self.ui.actStopCastClient, QtCore.SIGNAL("triggered()"), self.onStopCastClient)
        self.connect(self.ui.actStartExternalServers, QtCore.SIGNAL("triggered()"), self.onStartExternalServers)
        self.connect(self.ui.actStopExternalServers, QtCore.SIGNAL("triggered()"), self.onStopExternalServers)

        # Logging actions
        self.connect(self.ui.messageSourceCmbx, QtCore.SIGNAL("currentIndexChanged(int)"), self.onMessageFilterCmbxChanged)
        self.connect(self.ui.componentFilterCmbx, QtCore.SIGNAL("currentIndexChanged(int)"), self.onMessageFilterCmbxChanged)

        # Build actions
        self.connect(self.ui.actRunMake, QtCore.SIGNAL("triggered()"), self.onRunMake)
        self.connect(self.ui.actRunMakeInstall, QtCore.SIGNAL("triggered()"), self.onRunMakeInstall)
        self.connect(self.ui.actRunMakeClean, QtCore.SIGNAL("triggered()"), self.onRunMakeClean)
        self.connect(self.ui.actCancelBuild, QtCore.SIGNAL("triggered()"), self.onStopMake)
        self.connect(self.ui.actConfigureWithCMake, QtCore.SIGNAL("triggered()"), self.onRunCmakeConfig)
        self.connect(self.ui.actSynchronizeCode, QtCore.SIGNAL("triggered()"), self.onSynchronizeRemoteCode)

        self.connect(self.ui.actCtxShowBuildError, QtCore.SIGNAL("triggered()"), self.onEditBuildError)

        # Context menu actions for QTextEdit
        self.ui.buildLogfileTxt.contextActions.append(self.ui.actCtxShowBuildError)

        # Edit actions
        self.connect(self.ui.actEditUserSettings, QtCore.SIGNAL("triggered()"), self.onEditUserSettings)
        self.connect(self.ui.actEditCastEnvironment, QtCore.SIGNAL("triggered()"), self.onEditCastEnvironment)


    def _initContent(self):
        for fn in self._options.mruCfgCast:
            if fn.strip() == "": continue
            self.ui.clientConfigCmbx.addItem(self.makeConfigFileDisplay(fn), QtCore.QVariant(fn))
        self.ui.clientConfigCmbx.setToolTip(self._clientConfig)
        for fn in self._options.mruCfgHosts:
            if fn.strip() == "": continue
            self.ui.hostConfigCmbx.addItem(self.makeConfigFileDisplay(fn), QtCore.QVariant(fn))
        self.ui.hostConfigCmbx.setToolTip(self._hostConfig)

        self.updateCbLog4jHosts()
        val = self._options.getOption("log4jServerHost")
        if val != "": self.ui.cbLog4jServerHost.setEditText(val)
        val = self._options.getOption("log4jServerPort")
        if val != "": self.ui.edLog4jServerPort.setText(val)
        val = self._options.getOption("log4jServerOutfile")
        if val != "": self.ui.edLog4jOutfile.setText(val)

        log4jlevels = log4util.log4jlevels
        self.ui.log4jConsoleLevelCmbx.clear()
        self.ui.log4jFileLevelCmbx.clear()
        for i in log4jlevels:
            self.ui.log4jConsoleLevelCmbx.addItem(i)
            self.ui.log4jFileLevelCmbx.addItem(i)

        val = self._options.getOption('log4jConsoleLevel')
        if not val in log4jlevels: val = log4jlevels[0]
        self.ui.log4jConsoleLevelCmbx.setCurrentIndex(log4jlevels.index(val))

        val = self._options.getOption('log4jXmlFileLevel')
        if not val in log4jlevels: val = log4jlevels[-1]
        self.ui.log4jFileLevelCmbx.setCurrentIndex(log4jlevels.index(val))

        self._createLog4ServerModes()
        val = self._options.getOption('log4jServerMode')
        if not val: val = 0
        else: val = int(val)
        if val < 0 or val >= len(self.log4jModes): val = 0
        self.ui.cbLog4jMode.setCurrentIndex(val)

        self._createRemoteBuildModes()
        val = self._options.getOption('remoteBuildMode')
        if not val: val = 0
        else: val = int(val)
        if val < 0 or val >= len(self.buildModes): val = 0
        self.ui.cbRemoteBuildMode.setCurrentIndex(val)

        def ckint(field, default=0):
            try: return int(self._options.getOption(field)) % 3
            except: return default
        self.ui.ckShowFlushMsgs.setCheckState(ckint("ckShowFlushMsgs", 2))
        self.ui.ckShowInternalMsgs.setCheckState(ckint("ckShowInternalMsgs", 2))
        self.ui.ckAutoClearLog.setCheckState(ckint("ckAutoClearLog", 2))
        self.ui.ckAutoSyncCode.setCheckState(ckint("ckAutoSyncCode", 0))


    def _fillMessageFilterCombo(self):
        self.ui.messageSourceCmbx.clear()
        self.ui.messageSourceCmbx.addItem("All Sources")
        self.ui.messageSourceCmbx.addItem("All Processes")
        self.ui.messageSourceCmbx.addItem("All CAST Processes")
        self.ui.messageSourceCmbx.addItem("Cast Control Internal")
        machine = []
        def addsrc(srcid):
            if srcid.endswith("BUILD"): return
            m = ".".join(srcid.split('.')[:2])
            if not m in machine:
                machine.append(m)
                self.ui.messageSourceCmbx.addItem(m)
            self.ui.messageSourceCmbx.addItem(srcid)
        for p in self._manager.proclist: addsrc(p.srcid)
        for h in self._remoteHosts:
            for p in h.proclist: addsrc(p.srcid)

        self.ui.componentFilterCmbx.clear()
        self.ui.componentFilterCmbx.addItem("All components")
        self.ui.componentFilterCmbx.addItem("Selected components")


    def _applyMessageFilter(self):
        cmb = self.ui.messageSourceCmbx
        i = cmb.currentIndex()
        source = "%s" % cmb.itemText(i)
        cmb = self.ui.componentFilterCmbx
        i = cmb.currentIndex()
        components = "%s" % cmb.itemText(i)
        flt = ""
        if source == "All Sources": pass
        elif source == "Cast Control Internal":
            flt = """(m.source == "castcontrol")"""
        elif source == "All Processes":
            flt = """(m.source.startswith("process."))"""
        elif source == "All CAST Processes":
            flt = """(m.source.startswith("process.") and m.source.find(".cast-") > 0)"""
        else:
            flt = """(m.source.startswith(\'%s\'))""" % source

        # TODO: Group components by type (WM+TM, MG, ...); separate lists in selection dlg
        # TODO: subarch is important for WM/TM since those components don't have IDs in .cast!
        if components == "Selected components":
            con = []; coff = []
            for c in self.componentFilter:
                if c.cid == '': continue
                if c.status: con.append(c)
                else: coff.append(c)
            if len(con) < len(coff) + 1:
                comps = con; cond = "m.component in"
                extra = []
            else:
                comps = coff; cond = "not m.component in"
                extra = ["''"]
            # New log4j form for component id: 'SA.ID'; the old form was 'ID'.
            comps = extra + [
                "'%s.%s'" % (c.subarch, c.cid) for c in comps ] + [
                    "'%s.%s.main'" % (c.subarch, c.cid) for c in comps ]
            comps = ",".join(comps)
            if flt != "": flt = "(" + flt + ") and "
            flt = flt + "(" + cond + " [" + comps + "])"

        if self.ui.ckShowInternalMsgs.isChecked() and flt != "":
            if flt != "": flt = "(" + flt + ") or "
            flt = flt + """(m.source == "castcontrol")"""

        # LOGGER.log(flt)
        if flt == "": self.mainLog.setFilter(None)
        else:
            try: self.mainLog.setFilter(eval("lambda m: " + flt))
            except Exception as e:
                LOGGER.error("setFilter failed for: %s" % (flt))
                LOGGER.error("  error was: %s" % (e))
                self.mainLog.setFilter(None)

    def onMessageFilterCmbxChanged(self, index):
        self._applyMessageFilter()

    def makeConfigFileDisplay(self, fn):
        fn = "%s" % fn
        return "%s   @ %s" % (os.path.basename(fn), os.path.dirname(fn))

    # If the file is under the COGX_ROOT directory, make it relative
    def makeConfigFileRelPath(self, fn):
        wd = self._options.xe("${COGX_ROOT}")
        if wd.strip() == "": return fn
        rp = legacy.os_path_relpath("%s" % fn, wd)
        if rp.startswith(".."): return fn
        return rp

    def isNullFilename(self, fn):
        if fn == None: return True
        test = fn.lower().strip()
        if test == "": return True
        if test.startswith(NOFILE_FILENAME): return True
        if test.startswith("-"): return True
        return False

    @property
    def _clientConfig(self):
        cmb = self.ui.clientConfigCmbx
        i = cmb.currentIndex()
        fn = cmb.itemData(i)
        return fn.toString()

    @property
    def _hostConfig(self):
        cmb = self.ui.hostConfigCmbx
        i = cmb.currentIndex()
        fn = cmb.itemData(i)
        return "%s" % fn.toString()

    @property
    def _localhost(self):
        lhost = "%s" % self.ui.txtLocalHost.text()
        lhost = lhost.strip()
        return lhost

    @property
    def _log4j_start_server(self):
        return self.ui.cbLog4jMode.currentIndex() == 0

    @property
    def _log4j_use_server(self):
        return self.ui.cbLog4jMode.currentIndex() in [0, 1]

    def _createLog4ServerModes(self):
        self.log4jModes = ["Start and Connect to log4j Server",
                           "Connect to log4j Server", "Log to Console (No Server)"]
        self.ui.cbLog4jMode.clear()
        for item in self.log4jModes:
            self.ui.cbLog4jMode.addItem(item)

    @property
    def _log4jServerHost(self):
        val = "%s" % self.ui.cbLog4jServerHost.lineEdit().text()
        val = val.strip()
        if val == '' or val == 'localhost':
            val = 'localhost'
        return val

    @property
    def _log4jServerPort(self):
        val = "%s" % self.ui.edLog4jServerPort.text()
        val = val.strip()
        return val

    @property
    def _log4jServerOutfile(self):
        val = "%s" % self.ui.edLog4jOutfile.text()
        val = val.strip()
        return val

    @property
    def _log4jConsoleLevel(self):
        val = "%s" % self.ui.log4jConsoleLevelCmbx.currentText()
        val = val.strip()
        return val

    @property
    def _log4jXmlFileLevel(self):
        val = "%s" % self.ui.log4jFileLevelCmbx.currentText()
        val = val.strip()
        return val

    @property
    def _localBuildEnabled(self):
        val = self.ui.cbRemoteBuildMode.currentIndex()
        return val == 0 or val == 2

    @property
    def _remoteBuildEnabled(self):
        val = self.ui.cbRemoteBuildMode.currentIndex()
        return val == 1 or val == 2

    def _createRemoteBuildModes(self):
        self.buildModes = ["Local", "Remote", "Local & Remote"]
        self.ui.cbRemoteBuildMode.clear()
        for item in self.buildModes:
            self.ui.cbRemoteBuildMode.addItem(item)

    def _initLocalProcesses(self):
        self.procGroupA = CProcessGroup("External Servers")
        self.procGroupB = CProcessGroup("CAST Servers")
        self.procGroupC = CProcessGroup("CAST Client")

        self._manager.addProcess(procman.CProcess("cast-client", self._options.xe("${CMD_CAST_CLIENT}")))
        self.procGroupC.addProcess("cast-client")

        self.serverManager = CServerManager()
        fn = os.path.join(os.path.dirname(pconfig.__file__), "castservers.txt")
        self.serverManager.addServersFromFile(fn)
        fn = os.path.join(os.path.dirname(pconfig.__file__), "cogxservers.txt")
        self.serverManager.addServersFromFile(fn)
        self.wAppConfig.addServers(self.serverManager.servers)
        self.wAppConfig.updateHeader();

        # HACK: use CCastControlWnd as an interface for an external file editor
        pconfig.editors.TEXT_EDITOR = self

        for csi in self.serverManager.servers:
            if csi.group == 'B': self.procGroupB.addProcess(csi.name)
            elif csi.group == 'C': self.procGroupC.addProcess(csi.name)
            else: self.procGroupA.addProcess(csi.name)
            # the command will be evaluated at startup
            proc = procman.CProcess(csi.name, command=None)
            if csi.termWithSigInt:
                proc.killSignals = [signal.SIGINT]
            self._manager.addProcess(proc)

        if os.path.exists(self.fnhist):
            cfg = ConfigParser.RawConfigParser()
            cfg.read(self.fnhist)
            self.serverManager.loadServerConfig(cfg)

        p = procman.CProcess(procman.LOG4J_PROCESS, self._options.xe("${CMD_LOG4J_SERVER}"))
        p.messageProcessor = log4util.CLog4MessageProcessor()
        self._manager.addProcess(p)

        self.procBuild = procman.CProcess("BUILD", 'make [target]', workdir=self._options.xe("${COGX_BUILD_DIR}"))

        self.procBuild.allowTerminate = True
        self._manager.addProcess(self.procBuild)
        self._processModel.rootItem.addHost(self._manager)
        self.ui.processTree.expandAll()
        self.ui.processTree.resizeColumnToContents(0)

    def statusUpdate(self):
        self.mainLog.showFlush = self.ui.ckShowFlushMsgs.isChecked()
        self.mainLog.pullLogs()
        self.buildLog.pullLogs()
        for rpm in self._remoteHosts: #TODO: read in background, at most 1 per second
            rpm.updateProcessList()
            if self._pumpRemoteMessages: rpm.pumpRemoteMessages()

    def closeEvent(self, event):
        self._manager.stopAll()
        self._manager.stopReaderThread()
        time.sleep(0.2)
        def getitems(cmbx, count=30, hasNullFile=False):
            mx = cmbx.count()
            lst = []
            nullSelected = False
            for i in xrange(mx):
                fn = "%s" % cmbx.itemData(i).toString()
                if hasNullFile and self.isNullFilename(fn):
                    if i == 0: nullSelected = True
                    continue
                lst.append(fn)
            lst = lst[:count]
            if hasNullFile:
                if nullSelected: lst.insert(0, NOFILE_FILENAME)
                else: lst.append(NOFILE_FILENAME)
            return lst

        try:
            if self._userOptions.modified:
                self._userOptions.saveConfig()
        except Exception, e:
            print "Failed to save user configuration"
            print e
        try:
            self._options.mruCfgCast = getitems(self.ui.clientConfigCmbx)
            self._options.mruCfgHosts = getitems(self.ui.hostConfigCmbx, hasNullFile=True)
            self._options.setOption("log4jServerHost", self._log4jServerHost)
            self._options.setOption("log4jServerPort", self._log4jServerPort)
            self._options.setOption("log4jServerOutfile", self._log4jServerOutfile)
            self._options.setOption("log4jConsoleLevel", self._log4jConsoleLevel)
            self._options.setOption("log4jXmlFileLevel", self._log4jXmlFileLevel)
            self._options.setOption("log4jServerMode", self.ui.cbLog4jMode.currentIndex())
            self._options.setOption("ckShowFlushMsgs", self.ui.ckShowFlushMsgs.checkState())
            self._options.setOption("ckShowInternalMsgs", self.ui.ckShowInternalMsgs.checkState())
            self._options.setOption("ckAutoClearLog", self.ui.ckAutoClearLog.checkState())
            self._options.setOption("ckAutoSyncCode", self.ui.ckAutoSyncCode.checkState())
            self._options.setOption("remoteBuildMode", self.ui.cbRemoteBuildMode.currentIndex())

            cfg = ConfigParser.RawConfigParser()
            cfg.read(self.fnhist)
            self._options.saveHistory(cfg)
            self.serverManager.saveServerConfig(cfg)
            with open(self.fnhist, 'wb') as conffile:
                cfg.write(conffile)

            if not os.path.exists(self.fnconf):
                self._options.saveConfig(open(self.fnconf, 'w'))

        except Exception, e:
            print "Failed to save configuration"
            print e

    def runCleanupScript(self):
        script = []
        for stm in self._options.cleanupScript:
            stm = stm.split('#')[0]
            stm = stm.strip()
            if len(stm) < 1: continue
            script.append(self._options.xe(stm))
        for i,cmd in enumerate(script):
            # procman.runCommand(cmd, name="cleanup-cmd-%d" % (i+1))
            procman.xrun_wait(cmd)

    def startLocalProcesses(self, procGroup):
        LOGGER.log("Starting %s" % procGroup.name)
        for name in procGroup.processlist:
            csi = self.serverManager.getServerInfo(name)
            p = self._manager.getProcess(name)
            if not p:
                continue
            if not csi:
                p.start()
                time.sleep(0.3)
                continue
            if not csi.enabled:
                continue

            extenv = self._options.getExtendedEnviron(defaults=csi.getEnvVarScript())
            command = self._options.xe(csi.getCommand(), environ=extenv)
            workdir = self._options.xe(csi.workdir, environ=extenv) if csi.workdir != None else None
            params = csi.getParameters()
            if params:
                for k,v in params.items():
                    params[k] = self._options.xe(v, environ=extenv)
            p.start(command=command, params=params, workdir=workdir, allowTerminate=not csi.isServer)
            time.sleep(0.3)

    def startRemoteProcesses(self, procGroup):
        log4 = self._getLog4jConfig()
        log4config = "".join(open(log4.clientConfigFile).readlines())

        # XXX Processes could be started for each host separately;  (id=remotestart)
        # Each process on each host could be started separately;
        # ATM we start all processes on all hosts, so the UI can remain unchanged.
        for h in self._remoteHosts:
            h.agentProxy.setLog4jClientProperties(log4config)
            for p in h.proclist:
                if p.name == procman.LOG4J_PROCESS:
                    continue # log4j server is started separately
                if not p.name in procGroup.processlist:
                    continue
                p.start()


    def stopLocalProcesses(self, procGroup):
        LOGGER.log("Stopping %s" % procGroup.name)
        ts = []
        for name in procGroup.processlist:
            p = self._manager.getProcess(name)
            if p:
                t = CLambdaThread(p, lambda x: x.stop())
                ts.append(t)
                t.start()

        for t in ts: t.join()

    def stopRemoteProcesses(self, procGroup):
        for h in self._remoteHosts: # See <URL:#remotestart>
            for p in h.proclist:
                if p.name == procman.LOG4J_PROCESS:
                    continue # log4j server is started separately
                if not p.name in procGroup.processlist:
                    continue
                p.stop()

        #self.checkStopLog4jServer()


    def startServers(self):
        self._options.checkConfigFile()
        log4 = self._getLog4jConfig()
        hasServer = self._log4j_use_server
        log4.prepareClientConfig(console=(not hasServer), socketServer=hasServer)

        if self.ui.actEnableCleanupScript.isChecked():
            self.runCleanupScript()

        self.startLocalProcesses(self.procGroupB)
        self.startRemoteProcesses(self.procGroupB)


    def stopServers(self):
        self.stopLocalProcesses(self.procGroupB)
        self.stopRemoteProcesses(self.procGroupB)


    def onStartCastServers(self):
        try:
            QtGui.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
            self.ui.tabWidget.setCurrentWidget(self.ui.tabLogs)
            self.startServers()
            self.ui.processTree.expandAll()
        finally:
            QtGui.QApplication.restoreOverrideCursor()

    def onStopCastServers(self):
        try:
            QtGui.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
            self.stopServers()
            self.ui.processTree.expandAll()
        finally:
            QtGui.QApplication.restoreOverrideCursor()

    # Add components to filter; put the used components to the top of the list
    def addComponentFilters(self, components):
        for c in self.componentFilter: c.enabled = False
        for c in reversed(components):
            try:
                i = self.componentFilter.index(c)
                oc = self.componentFilter.pop(i)
                oc.enabled = c.enabled
                self.componentFilter.insert(0, oc)
            except ValueError:
                self.componentFilter.insert(0, c)

    # similar to buildConfigFile, but no file is generated
    def extractComponentsFromConfig(self):
        if self._clientConfig == None: clientConfig = ""
        else: clientConfig = "%s" % self._clientConfig
        clientConfig = clientConfig.strip()
        if clientConfig == "": return

        if self.isNullFilename(self._hostConfig): hostConfig = ""
        else: hostConfig = "%s" % self._hostConfig
        hostConfig = hostConfig.strip()
        lhost = self._localhost

        cfg = confbuilder.CCastConfig(self.configuredHosts)
        cfg.clearRules()
        if hostConfig != "": cfg.addRules(open(hostConfig, "r").readlines())

        cfg.prepareConfig(clientConfig)
        self.addComponentFilters(cfg.components)


    # build config file from rules in hostconfig
    def buildConfigFile(self):
        if self._clientConfig == None: clientConfig = ""
        else: clientConfig = "%s" % self._clientConfig
        clientConfig = clientConfig.strip()
        if self.isNullFilename(self._hostConfig): hostConfig = ""
        else: hostConfig = "%s" % self._hostConfig
        hostConfig = hostConfig.strip()
        lhost = self._localhost
        if clientConfig == "": return clientConfig
        if lhost == "" and hostConfig == "": return clientConfig
        LOGGER.log("LOCALHOST: %s" % lhost)

        self._tmpCastFile = tempfile.NamedTemporaryFile(suffix=".cast")
        cfg = confbuilder.CCastConfig(self.configuredHosts)
        cfg.clearRules()
        if hostConfig != "": cfg.addRules(open(hostConfig, "r").readlines())
        cfg.prepareConfig(clientConfig, self._tmpCastFile)
        self._tmpCastFile.flush()
        # don't self._tmpCastFile.close() or it will disappear

        self.addComponentFilters(cfg.components)
        return self._tmpCastFile.name


    def getConfiguredHosts(self):
        if self.isNullFilename(self._hostConfig): hostConfig = ""
        else: hostConfig = "%s" % self._hostConfig
        hostConfig = hostConfig.strip()
        lhost = self._localhost
        cfg = confbuilder.CCastConfig(self.configuredHosts)
        cfg.clearRules()
        if hostConfig == "" or not os.path.exists(hostConfig):
            msg = "Host configuration file (.hconf) not defined or it doesn't exist."
            LOGGER.warn(msg)
            raise UserWarning("HCONF: " + msg)
            return []

        cfg.addRules(open(hostConfig, "r").readlines())

        hlst = []
        for hid,haddr in self.configuredHosts.items():
            if hid[:4] == "127.": continue
            if haddr == "localhost" or haddr[:4] == "127.": continue
            # if haddr == lhost: continue # XXX Comment this line when testing and an agent is on localhost
            if not haddr in hlst: hlst.append(haddr)

        return hlst


    def discoverCastAgents(self):
        hosts = self.getConfiguredHosts()
        LOGGER.log("Hosts: %s" % (hosts))
        port = castagentsrv.SLAVE_PORT # TODO: user setting, maybe per-host setting
        working = castagentsrv.discoverRemoteHosts(hosts, port)
        LOGGER.log("Cast agents: %s" % (working))

        return working


    def onStartCastClient(self):
        log4 = self._getLog4jConfig()
        hasServer = self._log4j_use_server
        try:
            QtGui.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
            log4.prepareClientConfig(console=(not hasServer), socketServer=hasServer)

            if self.ui.ckAutoClearLog.isChecked():
                self.mainLog.clearOutput()
            self._options.checkConfigFile()
            p = self._manager.getProcess("cast-client")
            if p != None:
                self.ui.tabWidget.setCurrentWidget(self.ui.tabLogs)
                try:
                    confname = self.buildConfigFile()
                    if confname != None and confname != "":
                        p.start( params = { "CAST_CONFIG": confname } )
                except Exception as e:
                    LOGGER.error("%s" % e)
            # if p != None: p.start( params = { "CAST_CONFIG": self._options.mruCfgCast[0] } )
        finally:
            QtGui.QApplication.restoreOverrideCursor()


    def onStopCastClient(self):
        try:
            QtGui.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
            p = self._manager.getProcess("cast-client")
            if p != None: p.stop()
            #self.checkStopLog4jServer()
        finally:
            QtGui.QApplication.restoreOverrideCursor()


    def onLog4jHostCbItemActivated(self, index):
        if index < 0: return
        cb = self.ui.cbLog4jServerHost
        value = "%s" % cb.itemData(index).toString()
        cb.setEditText(value)


    def updateCbLog4jHosts(self):
        text = self._log4jServerHost
        cb = self.ui.cbLog4jServerHost
        cb.blockSignals(True)
        cb.clear()
        for k,v in sorted(self.configuredHosts.items()):
            if self.configuredHosts.isLocalHost(k):
                if k != "localhost": continue
            cb.addItem("%s (%s)" % (k, v), QtCore.QVariant(k))
        self.ui.cbLog4jServerHost.setEditText(text)
        cb.blockSignals(False)


    def _getLog4jConfig(self):
        try:
            log4 = log4util.CLog4Config()
            log4.setXmlLogFilename(self._log4jServerOutfile)
            log4.setServerConsoleLevel(self._log4jConsoleLevel)
            log4.setServerXmlFileLevel(self._log4jXmlFileLevel)
            log4.setServerPort(self._log4jServerPort)
            log4.serverHost = self.configuredHosts.expandHostName(self._log4jServerHost)
            return log4
        except Exception as e:
            dlg = QtGui.QErrorMessage(self)
            dlg.setModal(True)
            dlg.showMessage("Had some problems preparing the log file.\n%s" % e)


    def startLog4jServer(self):
        startServer = self._log4j_start_server
        if not startServer:
            return

        # TODO: start the server only if it's not running, yet

        log4 = self._getLog4jConfig()
        if self.configuredHosts.isLocalHost(log4.serverHost):
            p = self._manager.getProcess(procman.LOG4J_PROCESS)
            if p != None:
                log4.prepareServerConfig()
                LOGGER.log("Log4j STARTING 2")
                p.start( params = {
                    "LOG4J_PORT": log4.serverPort,
                    "LOG4J_SERVER_CONFIG": log4.serverConfigFile
                    })
                time.sleep(1) # give the server time to start before the others start
                self.log4jStartedOn = "LOCAL"
                return

        # start Log4j remotely
        haddr = self.configuredHosts.expandHostName(log4.serverHost)
        for h in self._remoteHosts:
            if h.address != haddr: continue
            p = h.getProcess(procman.LOG4J_PROCESS)
            if p != None:
                log4.prepareServerConfig()
                conf = open(log4.serverConfigFile).read()
                h.agentProxy.setLog4jServerProperties(int(log4.serverPort), conf)
                p.start()
                time.sleep(1) # give the server time to start before the others start
                self.log4jStartedOn = h.address
                return


    # Stop log4j server if it was started (and all other processes are stopped)
    def checkStopLog4jServer(self):
        if not self.log4jStartedOn:
            return

        # TODO: (maybe) Check if everything else is stopped

        if self.log4jStartedOn == "LOCAL":
            p = self._manager.getProcess(procman.LOG4J_PROCESS)
            if p:
                p.stop()
                return

        for h in self._remoteHosts:
            if h.address != self.log4jStartedOn: continue
            p = h.getProcess(procman.LOG4J_PROCESS)
            if p:
                p.stop()
                return


    def onStartExternalServers(self):
        self._options.checkConfigFile()
        log4 = self._getLog4jConfig()

        try:
            QtGui.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
            self.startLog4jServer()

            hasServer = self._log4j_use_server
            log4.prepareClientConfig(console=(not hasServer), socketServer=hasServer)

            self.startLocalProcesses(self.procGroupA)
        finally:
            QtGui.QApplication.restoreOverrideCursor()


    def onStopExternalServers(self):
        try:
            QtGui.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
            self.stopLocalProcesses(self.procGroupA)
            self.checkStopLog4jServer()
        finally:
            QtGui.QApplication.restoreOverrideCursor()


    def _checkBuidDir(self):
        workdir=self._options.xe("${COGX_BUILD_DIR}")
        if not os.path.exists(workdir):
            dlg = QtGui.QErrorMessage(self)
            dlg.setModal(True)
            dlg.showMessage("The build directory doesn't exist. Run 'Build.Configure With CMake'.")
            return False
        return True

    def _runRemoteBuild(self, target):
        if not self._remoteBuildEnabled:
            return

        if self.ui.ckAutoSyncCode.isChecked():
            try:
                QtGui.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
                self.syncRemoteCode()
            finally:
                QtGui.QApplication.restoreOverrideCursor()

        for h in self._remoteHosts:
            h.agentProxy.startBuild(target)

    def _runLocalBuild(self, target):
        if not self._localBuildEnabled or not self._checkBuidDir():
            return

        p = self._manager.getProcess("BUILD")
        if p != None:
            self.ui.tabWidget.setCurrentWidget(self.ui.tabBuildLog)
            self.buildLog.clearOutput()
            if not self.buildLog.log.hasSource(p): self.buildLog.log.addSource(p)
            p.start(params={"target": target})

    def onRunMake(self):
        target = ""
        self._runLocalBuild(target)
        self._runRemoteBuild(target)

    def onRunMakeInstall(self):
        target = "install"
        self._runLocalBuild(target)
        self._runRemoteBuild(target)

    def onRunMakeClean(self):
        target = "clean"
        self._runLocalBuild(target)
        self._runRemoteBuild(target)

        # CMake can't run "ant clean" (can't add a clean dependency), so we run it here 
        # TODO: chould check in CMakeCache if DO_ANT is enabled
        #root = self._options.xe("${COGX_ROOT}")
        #procman.xrun_wait("ant clean", root)

    def _stopRemoteBuild(self):
        if not self._remoteBuildEnabled:
            return

        for h in self._remoteHosts:
            h.agentProxy.stopBuild()

    def onStopMake(self):
        p = self._manager.getProcess("BUILD")
        if p and p.isRunning():
            p.stop()

        self._stopRemoteBuild()

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

    def onRunCmakeConfig(self):
        root = self._options.xe("${COGX_ROOT}")
        bdir = self._options.xe("${COGX_BUILD_DIR}")
        bcmc = os.path.join(bdir, "CMakeCache.txt")
        if not self._checkMakeCache(root, bcmc): return
        cmd = 'cmake-gui %s' % root
        procman.xrun_wait(cmd, bdir)


    def syncRemoteCode(self):
        lhost = self._localhost
        sync = rasync.CRemoteSync()

        for h in self._remoteHosts:
            sync.rsync(lhost, h)


    def onSynchronizeRemoteCode(self):
        try:
            QtGui.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
            self.syncRemoteCode()
        finally:
            QtGui.QApplication.restoreOverrideCursor()

    def on_btEditFilterComponents_clicked(self, valid=True):
        if not valid: return

        try:
            self.extractComponentsFromConfig()
        except Exception as e:
            LOGGER.error("%s" % e)
            return

        dlg = CSelectComponentsDlg(self)
        dlg.setComponentList(self.componentFilter)
        rv = dlg.exec_()
        if rv == QtGui.QDialog.Accepted:
            self._applyMessageFilter()

    def on_btClearMainLog_clicked(self, valid=True):
        if not valid: return
        self.mainLog.clearOutput()

    def on_ckShowFlushMsgs_stateChanged(self, value):
        self.mainLog.rereadLogs()

    def on_ckShowInternalMsgs_stateChanged(self, value):
        self._applyMessageFilter()

    def editFileInternal(self, filename, line=None):
        LOGGER.log("internal-editor: %s" % (filename))
        dlg = CTextEditor(self)
        dlg.editFile(filename, line)

    def editFile(self, filename, line=None):
        if filename == None: return
        filename = "%s" % filename
        if filename.strip() == "": return

        cmd = self._userOptions.textEditCmd
        if cmd.startswith("internal"):
            self.editFileInternal(filename, line)
        else:
            mo = re.search("%l(\[([^\]]+)\])?", cmd)
            if mo != None:
                if line == None: lexpr = ""
                else: lexpr = "%s%d" % (mo.group(2), line)
                cmd = cmd[:mo.start()] + lexpr + cmd[mo.end():]
            pid = procman.xrun(cmd % filename)
            if pid == None:
                LOGGER.error("editFile(): failed to execute '%s' for '%s'" % (cmd, filename))
                self.editFileInternal(filename, line)

    def onEditUserSettings(self):
        fn = self._userOptions.configFile
        self.editFile(fn)

    def onEditCastEnvironment(self):
        fn = self.fnconf
        self.editFile(fn)

    def on_btEditClientConfig_clicked(self, valid=True):
        if not valid: return
        self.editFile(self._clientConfig)

    def on_btEditHostConfig_clicked(self, valid=True):
        if not valid: return
        if not self.isNullFilename(self._hostConfig):
            self.editFile(self._hostConfig)

    def on_btDetectLocalHost_clicked(self, valid=True):
        if not valid: return
        ip = network.get_ip_address()
        if ip != None:
            self.ui.txtLocalHost.setText(ip.strip())
            self._options.setOption("localhost", ip.strip())
            # TODO: setLocalhost should also be called onEditChanged()
            self.configuredHosts.setLocalhost(ip.strip())

    def on_btDiscoverRemote_clicked(self, valid=True):
        if not valid: return
        try:
            QtGui.QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
            agents = self.discoverCastAgents()
        except UserWarning as uw:
            rv = QtGui.QMessageBox.information(self, "Discover Agents", "%s" % uw)
            return
        finally:
            QtGui.QApplication.restoreOverrideCursor()

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
                self.mainLog.log.addSource(remoteproc.CRemoteMessageSource(rpm))

        self.ui.processTree.expandAll()
        self._fillMessageFilterCombo()
        self.updateCbLog4jHosts()


    def onShowEnvironment(self):
        cmd = "bash -c env"
        # procman.runCommand(cmd, name="ENV")
        procman.xrun_wait(cmd)

    def onStartTerminal(self):
        root = self._options.xe("${COGX_ROOT}")
        cmd = self._userOptions.terminalCmd
        if cmd.find("%s") > 0: cmd = cmd % root
        procman.xrun(cmd, env=self._options.environ)

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
        self.ui.clientConfigCmbx.setToolTip(self._clientConfig)

    def onClientConfigChanged(self, index):
        if index < 1: return
        fn = self._clientConfig
        self._ComboBox_SelectMru(self.ui.clientConfigCmbx, index,
                self.makeConfigFileDisplay(fn), QtCore.QVariant(fn))
        self.ui.clientConfigCmbx.setToolTip(self._clientConfig)

    def onBrowseHostConfig(self):
        qfd = QtGui.QFileDialog
        fn = "%s" % qfd.getOpenFileName(
            self, self.ui.actOpenHostConfig.text(),
            "", "Component Host Config (*.hconf)")
        if fn == None or len(fn) < 2:
            return
        elif self.isNullFilename(fn):
            self._ComboBox_AddMru(self.ui.hostConfigCmbx, NOFILE_FILENAME, NOFILE_FILENAME)
        else:
            fn = self.makeConfigFileRelPath(fn)
            self._ComboBox_AddMru(self.ui.hostConfigCmbx,
                    self.makeConfigFileDisplay(fn), QtCore.QVariant(fn))
        self.ui.hostConfigCmbx.setToolTip(self._hostConfig)

    def onHostConfigChanged(self, index):
        if index < 1: return
        fn = "%s" % self._hostConfig
        if self.isNullFilename(fn):
            self._ComboBox_SelectMru(self.ui.hostConfigCmbx, index, NOFILE_FILENAME, NOFILE_FILENAME)
        else:
            self._ComboBox_SelectMru(self.ui.hostConfigCmbx, index,
                    self.makeConfigFileDisplay(fn), QtCore.QVariant(fn))
        self.ui.hostConfigCmbx.setToolTip(self._hostConfig)

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
    rv = app.exec_()
    sys.exit(rv)


if __name__ == "__main__":
    guiMain()

