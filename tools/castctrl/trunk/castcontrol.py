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
from selectcomponentdlg import CSelectComponentsDlg
from textedit import CTextEditor
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
        self._userOptions.loadConfig()
        self.componentFilter = []

        # move option values to UI
        self.ui.txtLocalHost.setText(self._options.getOption("localhost"))
        val = self._options.getOption("log4jServerPort")
        if val != "": self.ui.edLo4jServerPort.setText(val)
        val = self._options.getOption("log4jServerOutfile")
        if val != "": self.ui.edLog4jOutfile.setText(val)

        self._manager = procman.CProcessManager()
        self._remoteHosts = []
        self._pumpRemoteMessages = True

        root = self._options.xe("${COGX_ROOT}")
        if len(root) > 64: root = "..." + root[-64:]
        self.setWindowTitle("CAST Control - " + root)

        # XXX keep the old interface, just in case, but hide it
        self.ui.tabWidget.removeTab(self.ui.tabWidget.indexOf(self.ui.tabOldInterface))

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

        # Event connections
        self.connect(self.ui.actQuit, QtCore.SIGNAL("triggered()"), self.close)
        self.connect(self.ui.actShowEnv, QtCore.SIGNAL("triggered()"), self.onShowEnvironment)

        # Config actions
        self.connect(self.ui.actOpenClientConfig, QtCore.SIGNAL("triggered()"), self.onBrowseClientConfig)
        self.connect(self.ui.actOpenPlayerConfig, QtCore.SIGNAL("triggered()"), self.onBrowsePlayerConfig)
        self.connect(self.ui.actOpenHostConfig, QtCore.SIGNAL("triggered()"), self.onBrowseHostConfig)
        self.connect(self.ui.actStartTerminal, QtCore.SIGNAL("triggered()"), self.onStartTerminal)
        self.connect(self.ui.clientConfigCmbx, QtCore.SIGNAL("currentIndexChanged(int)"), self.onClientConfigChanged)
        self.connect(self.ui.playerConfigCmbx, QtCore.SIGNAL("currentIndexChanged(int)"), self.onPlayerConfigChanged)
        self.connect(self.ui.hostConfigCmbx, QtCore.SIGNAL("currentIndexChanged(int)"), self.onHostConfigChanged)

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
        self.connect(self.ui.actConfigureWithCMake, QtCore.SIGNAL("triggered()"), self.onRunCmakeConfig)

        self.connect(self.ui.actCtxShowBuildError, QtCore.SIGNAL("triggered()"), self.onEditBuildError)

        # Context menu actions for QTextEdit
        self.ui.buildLogfileTxt.contextActions.append(self.ui.actCtxShowBuildError)

        # Edit actions
        self.connect(self.ui.actEditUserSettings, QtCore.SIGNAL("triggered()"), self.onEditUserSettings)
        self.connect(self.ui.actEditCastEnvironment, QtCore.SIGNAL("triggered()"), self.onEditCastEnvironment)

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

        log4jlevels = ['ALL', 'TRACE', 'DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL', 'OFF']
        for i in log4jlevels:
            self.ui.log4jConsoleLevelCmbx.addItem(i)
            self.ui.log4jFileLevelCmbx.addItem(i)

        val = self._options.getOption('log4jConsoleLevel')
        if not val in log4jlevels: val = log4jlevels[0]
        self.ui.log4jConsoleLevelCmbx.setCurrentIndex(log4jlevels.index(val))

        val = self._options.getOption('log4jXmlFileLevel')
        if not val in log4jlevels: val = log4jlevels[-1]
        self.ui.log4jFileLevelCmbx.setCurrentIndex(log4jlevels.index(val))

        def ckint(field, default=0):
            try: return int(self._options.getOption(field)) % 3
            except: return default
        self.ui.ckShowFlushMsgs.setCheckState(ckint("ckShowFlushMsgs", 2))
        self.ui.ckShowInternalMsgs.setCheckState(ckint("ckShowInternalMsgs", 2))
        self.ui.ckRunPeekabot.setCheckState(ckint("ckRunPeekabot", 2))
        self.ui.ckRunPlayer.setCheckState(ckint("ckRunPlayer", 2))
        self.ui.ckRunLog4jServer.setCheckState(ckint("ckRunLog4jServer", 2))
        self.ui.ckRunDisplaySrv.setCheckState(ckint("ckRunDisplaySrv", 2))
        self.ui.ckRunAbducerServer.setCheckState(ckint("ckRunAbducerServer", 2))
        self.ui.ckAutoClearLog.setCheckState(ckint("ckAutoClearLog", 2))

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
            comps = extra + [ "'" + c.cid + "'" for c in comps ]
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

    @property
    def _log4jServerPort(self):
        val = "%s" % self.ui.edLo4jServerPort.text()
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

    def _initLocalProcesses(self):
        self._manager.addProcess(procman.CProcess("cast-java", self._options.xe("${CMD_JAVA_SERVER}")))
        self._manager.addProcess(procman.CProcess("cast-cpp", self._options.xe("${CMD_CPP_SERVER}")))
        self._manager.addProcess(procman.CProcess("cast-python", self._options.xe("${CMD_PYTHON_SERVER}")))
        self._manager.addProcess(procman.CProcess("cast-client", self._options.xe("${CMD_CAST_CLIENT}")))
        self._manager.addProcess(procman.CProcess("display", self._options.xe("${CMD_DISPLAY_SERVER}")))
        self._manager.addProcess(procman.CProcess("abducer", self._options.xe("${CMD_ABDUCER_SERVER}")))
        self._manager.addProcess(procman.CProcess("player", self._options.xe("${CMD_PLAYER}")))
        self._manager.addProcess(procman.CProcess("peekabot", self._options.xe("${CMD_PEEKABOT}")))
        self._manager.addProcess(procman.CProcess("log4jServer", self._options.xe("${CMD_LOG4J_SERVER}")))

        self.procBuild = procman.CProcess("BUILD", 'make [target]', workdir=self._options.xe("${COGX_BUILD_DIR}"))

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
            if self._userOptions.modified:
                self._userOptions.saveConfig()
        except Exception, e:
            print "Failed to save user configuration"
            print e
        try:
            self._options.mruCfgCast = getitems(self.ui.clientConfigCmbx)
            self._options.mruCfgPlayer = getitems(self.ui.playerConfigCmbx)
            self._options.mruCfgHosts = getitems(self.ui.hostConfigCmbx)
            self._options.setOption("log4jServerPort", self._log4jServerPort)
            self._options.setOption("log4jServerOutfile", self._log4jServerOutfile)
            self._options.setOption("log4jConsoleLevel", self._log4jConsoleLevel)
            self._options.setOption("log4jXmlFileLevel", self._log4jXmlFileLevel)
            self._options.setOption("ckShowFlushMsgs", self.ui.ckShowFlushMsgs.checkState())
            self._options.setOption("ckShowInternalMsgs", self.ui.ckShowInternalMsgs.checkState())
            self._options.setOption("ckRunPeekabot", self.ui.ckRunPeekabot.checkState())
            self._options.setOption("ckRunPlayer", self.ui.ckRunPlayer.checkState())
            self._options.setOption("ckRunLog4jServer", self.ui.ckRunLog4jServer.checkState())
            self._options.setOption("ckRunDisplaySrv", self.ui.ckRunDisplaySrv.checkState())
            self._options.setOption("ckRunAbducerServer", self.ui.ckRunAbducerServer.checkState())
            self._options.setOption("ckAutoClearLog", self.ui.ckAutoClearLog.checkState())
            self._options.saveHistory(open(self.fnhist, 'w'))
            if not os.path.exists(self.fnconf):
                self._options.saveConfig(open(self.fnconf, 'w'))
        except Exception, e:
            print "Failed to save configuration"
            print e

    def getServers(self, manager):
        srvs = []
        p = manager.getProcess("cast-java")
        if p != None: srvs.append(p)
        p = manager.getProcess("cast-cpp")
        if p != None: srvs.append(p)
        p = manager.getProcess("cast-python")
        if p != None: srvs.append(p)
        return srvs

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

    def startServers(self):
        self._options.checkConfigFile()
        if self.ui.actEnableCleanupScript.isChecked():
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
    def onStartCastServers(self):
        self.ui.tabWidget.setCurrentWidget(self.ui.tabLogs)
        self.startServers()

    def onStopCastServers(self):
        self.stopServers()

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

        if self._hostConfig == None: hostConfig = ""
        else: hostConfig = "%s" % self._hostConfig
        hostConfig = hostConfig.strip()
        lhost = self._localhost

        cfg = confbuilder.CCastConfig()
        cfg.clearRules()
        if lhost != "": cfg.setLocalhost(lhost)
        if hostConfig != "": cfg.addRules(open(hostConfig, "r").readlines())
        cfg.prepareConfig(clientConfig)
        self.addComponentFilters(cfg.components)

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
        # don't self._tmpCastFile.close() or it will disappear

        self.addComponentFilters(cfg.components)
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


    def onStartCastClient(self):
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

    def onStopCastClient(self):
        p = self._manager.getProcess("cast-client")
        if p != None: p.stop()

    def _prepareLogFile(self, logfile):
        try:
            if not os.path.exists(logfile):
                fdir, fn = os.path.split(logfile)
                if not os.path.exists(fdir):
                    os.makedirs(fdir)
            f = open(logfile, 'w')
            head = self._options.getSection("LOG4J.SimpleSocketServer.XMLLayout.head")
            f.write("\n".join(head))
            f.close()
        except Exception as e:
            dlg = QtGui.QErrorMessage(self)
            dlg.setModal(True)
            dlg.showMessage("Had some problems preparing the log file.\n%s" % e)

    def prepareLog4jServer(self, fnameSrvConf):
        f = open(fnameSrvConf, "w")
        conf = self._options.getSection("LOG4J.SimpleSocketServer.conf")
        f.write("\n".join(conf))
        level = self._log4jConsoleLevel
        if level != 'OFF':
            conf = self._options.getSection("LOG4J.SimpleSocketServer.console")
            for ln in conf:
                ln = ln.replace('${LEVEL}', level)
                f.write(ln); f.write('\n')

        level = self._log4jXmlFileLevel
        logfile = self._log4jServerOutfile
        if level != 'OFF':
            if logfile == '': logfile = 'logs/cast-log.xml'
            logfile = os.path.abspath(logfile)
            self._prepareLogFile(logfile)
            conf = self._options.getSection("LOG4J.SimpleSocketServer.xmlfile")
            for ln in conf:
                ln = ln.replace('${LEVEL}', level)
                ln = ln.replace('${LOGFILE}', logfile)
                f.write(ln); f.write('\n')

        f.close()

    def onStartExternalServers(self):
        self._options.checkConfigFile()
        if not self.ui.ckRunLog4jServer.isChecked():
            pass # TODO: ln -s log4j configuration for client: one with server, one w/o
        else:
            p = self._manager.getProcess("log4jServer")
            if p != None:
                LOGGER.log("Log4j STARTING 2")
                conf = "cctmp.SimpleSocketServer.conf"
                self.prepareLog4jServer(conf)
                p.start( params = {
                    "LOG4J_PORT": self._log4jServerPort,
                    "LOG4J_SERVER_CONFIG": conf
                    })

        if self.ui.ckRunPlayer.isChecked():
            p = self._manager.getProcess("player")
            if p != None: p.start( params = { "PLAYER_CONFIG": self._playerConfig } )

        if self.ui.ckRunPeekabot.isChecked():
            p = self._manager.getProcess("peekabot")
            if p != None: p.start()

        if self.ui.ckRunDisplaySrv.isChecked():
            p = self._manager.getProcess("display")
            if p != None: p.start()

        if self.ui.ckRunAbducerServer.isChecked():
            p = self._manager.getProcess("abducer")
            if p != None: p.start()

    def onStopExternalServers(self):
        p = self._manager.getProcess("player")
        if p != None: p.stop()
        p = self._manager.getProcess("peekabot")
        if p != None: p.stop()
        p = self._manager.getProcess("log4jServer")
        if p != None: p.stop()
        p = self._manager.getProcess("display")
        if p != None: p.stop()
        p = self._manager.getProcess("abducer")
        if p != None: p.stop()

    def _checkBuidDir(self):
        workdir=self._options.xe("${COGX_BUILD_DIR}")
        if not os.path.exists(workdir):
            dlg = QtGui.QErrorMessage(self)
            dlg.setModal(True)
            dlg.showMessage("The build directory doesn't exist. Run 'Config...'")
            return False
        return True

    def onRunMake(self):
        if not self._checkBuidDir(): return
        p = self._manager.getProcess("BUILD")
        if p != None:
            self.ui.tabWidget.setCurrentWidget(self.ui.tabBuildLog)
            self.buildLog.clearOutput()
            if not self.buildLog.log.hasSource(p): self.buildLog.log.addSource(p)
            p.start(params={"target": ""})
            # p.start()

    def onRunMakeInstall(self):
        if not self._checkBuidDir(): return
        p = self._manager.getProcess("BUILD")
        if p != None:
            self.ui.tabWidget.setCurrentWidget(self.ui.tabBuildLog)
            self.buildLog.clearOutput()
            if not self.buildLog.log.hasSource(p): self.buildLog.log.addSource(p)
            p.start(params={"target": "install"})

    def onRunMakeClean(self):
        if not self._checkBuidDir(): return
        p = self._manager.getProcess("BUILD")
        if p != None:
            self.ui.tabWidget.setCurrentWidget(self.ui.tabBuildLog)
            self.buildLog.clearOutput()
            if not self.buildLog.log.hasSource(p): self.buildLog.log.addSource(p)
            p.start(params={"target": "clean"})

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

    def on_btEditFilterComponents_clicked(self, valid=True):
        if not valid: return
        self.extractComponentsFromConfig()
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
                self.mainLog.log.addSource(remoteproc.CRemoteMessageSource(rpm))

        self.ui.processTree.expandAll()
        self._fillMessageFilterCombo()


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

