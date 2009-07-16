#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009 
import sys, traceback, Ice
import threading, time

import modice
import icemodule.castcontrol.CastAgent as CastAgent
from core import messages, options, procman

LOGGER = messages.CInternalLogger()
procman.LOGGER = LOGGER

class CMonitor(threading.Thread):
    def __init__(self, *args, **kwds):
        super(CMonitor, self).__init__(*args, **kwds)
        # Similar to CCastControlWnd
        self.keeprunning = True
        self._options = options.CCastOptions()
        self._options.loadConfig("castcontrol.conf")
        self._options.configEnvironment()
        self._manager = procman.CProcessManager()
        self._initLocalProcesses()

    def _initLocalProcesses(self):
        self._manager.addProcess(procman.CProcess("server-java", options.xe("${CMD_JAVA_SERVER}")))
        self._manager.addProcess(procman.CProcess("server-cpp", options.xe("${CMD_CPP_SERVER}")))
        self._manager.addProcess(procman.CProcess("client", options.xe("${CMD_CAST_CLIENT}")))
        self._manager.addProcess(procman.CProcess("player", options.xe("${CMD_PLAYER}")))
        # TODO: local message sources/mergers for each process
        #for proc in self._manager.proclist:
        #    self.mainLog.log.addSource(proc)

    def run(self):
        self.keeprunning = True
        while self.keeprunning:
            self._manager.communicate()
            time.sleep(0.1)

class CAgentI(CastAgent.Agent):
    def __init__(self, monitor):
        self.monitor = monitor
        self.logs = {}

    def getProcessList(self, current=None):
        LOGGER.log("retireveing process list")
        plist = self.monitor._manager.proclist
        return [p.name for p in plist]

    def readMessages(self, processName, startTime, current=None):
        p = self.monitor._manager.getProcess(processName)
        if p == None:
            if processName == "LOGGER": p = LOGGER
            else: return []
        if self.logs.has_key(processName): log = self.logs[processName]
        else:
            log = messages.CLogMerger()
            log.addSource(p)
            self.logs[processName] = log
        if log == None: return []
        log.merge()
        msgs = [m for m in log.messages]
        res = [msg.getText() for msg in msgs]
        return res

    def startProcess(self, processName, current=None):
        LOGGER.log("starting %s" % processName)
        p = self.monitor._manager.getProcess(processName)
        if p != None: p.start()
        return 1 if p != None else 0

    def stopProcess(self, processName, current=None):
        LOGGER.log("stopping %s" % processName)
        p = self.monitor._manager.getProcess(processName)
        if p != None: p.stop()
        return 1 if p != None else 0

class Server(Ice.Application):
    def __init__(self, monitor):
        self.monitor = monitor

    def run(self, args):
        self.monitor.start()
        ic = Ice.initialize(sys.argv)
        adapter = ic.createObjectAdapterWithEndpoints("CastAgentAdapter", "tcp -p 10000:udp -p 10000")
        obj = CAgentI(self.monitor)
        adapter.add(obj, ic.stringToIdentity("CastAgent"))
        adapter.activate()
        ic.waitForShutdown() # No event loop here, just waiting for shutdown()
        self.monitor.keeprunning = False
        return 0

app = Server(CMonitor(name="CAST Process Monitor"))
status = app.main(sys.argv)
sys.exit(status)

