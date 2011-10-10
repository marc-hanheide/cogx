#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009 
import sys, os, traceback, Ice
import threading, time
import stat

import modice
import icemodule.castcontrol.CastAgent as CastAgent
# import icemodule.castcontrol.ProcessInfo as ProcessInfo
from core import messages, logger, options, procman, rasync

LOGGER = logger.get()

# Default port for the Ice server of a CastAgent.
SLAVE_PORT=7832

RSYNC_DAEMON = rasync.CRSyncDaemonConfig()

# Ice servant
class CAgentI(CastAgent.Agent):
    logDir = os.path.abspath("./logs")
    clientfile = os.path.join(logDir, "ccatmp.log4client.conf")
    serverfile = os.path.join(logDir, "ccatmp.log4server.conf")

    def __init__(self, processManager, options):
        self.manager = processManager
        self.options = options
        self.logs = {}
        self.log4jport = 48143

    def getProcessList(self, current=None):
        # LOGGER.log("Retrieveing process list")
        plist = self.manager.proclist
        procs = [CastAgent.ProcessInfo(name=p.name, status=p.status, error=p.error) for p in plist]
        return procs

    # NOTE: This procedure is implemented for a scenario with only one master castctrl.
    # TODO: It might be expensive to handle each process separately; get messages from multiple processes
    def readMessages(self, processName, current=None):
        """
        Retrieve new messages for the process. Up to 100 messages are retrieved at a time.
        Each message can be retrieved at most once.
        """
        # LOGGER.log("Gathering messages")
        p = self.manager.getProcess(processName)
        if p == None:
            if processName == "LOGGER": p = LOGGER
            else: return []
        if self.logs.has_key(processName):
            log = self.logs[processName]
        else:
            log = messages.CLogMerger()
            log.addSource(p)
            self.logs[processName] = log
        if log == None: return []
        try:
            log.merge()
            msgs = log.getNewMessages(100)
            # Convert messages to transport format (ice) and return them
            res = [
                CastAgent.CastMessage(time=msg.time, msgtype=msg.msgtype, message=msg.message)
                for msg in msgs]
            return res
        except: pass
        return []

    def startProcess(self, processName, current=None):
        # LOGGER.log("Starting %s" % processName)
        p = self.manager.getProcess(processName)
        if not p:
            return 0
        params = None
        workdir = None

        if processName == procman.LOG4J_PROCESS:
            params = {
                      "LOG4J_PORT": self.log4jport,
                      "LOG4J_SERVER_CONFIG": CAgentI.serverfile
                     }
            workdir = CAgentI.logDir

        p.start(params=params, workdir=workdir)
        return 1

    def stopProcess(self, processName, current=None):
        # LOGGER.log("Stopping %s" % processName)
        p = self.manager.getProcess(processName)
        if p != None: p.stop()
        return 1 if p != None else 0

    def setLog4jClientProperties(self, propText, current=None):
        # based on log4jutil.prepareClientConfig()
        # WARNING: if the client and the server are running from the same directory
        # the link to log4j properties will switch between two files
        logPropLink = "log4j.properties"
        if not os.path.exists(CAgentI.logDir):
            os.makedirs(CAgentI.logDir)
        f = open(CAgentI.clientfile, 'w')
        f.write(propText)
        f.close()
        if os.path.exists(logPropLink):
            if not os.path.islink(logPropLink):
                os.rename(logPropLink, os.tempnam(CAgentI.logDir, "cli." + logPropLink))
            os.remove(logPropLink)
        os.symlink(CAgentI.clientfile, logPropLink)

    def setLog4jServerProperties(self, port, propText, current=None):
        self.log4jport = port
        logPropLink = "log4j.properties"
        if not os.path.exists(CAgentI.logDir):
            os.makedirs(CAgentI.logDir)
        f = open(CAgentI.serverfile, 'w')
        f.write(propText)
        f.close()
        serverLink = os.path.join(CAgentI.logDir, logPropLink)
        if os.path.exists(serverLink):
            if not os.path.islink(serverLink):
                os.rename(serverLink, os.tempnam(CAgentI.logDir, "srv." + logPropLink))
            os.remove(serverLink)
        os.symlink(CAgentI.serverfile, serverLink)

    def getCmakeCache(self, current=None):
        if not self._checkBuildDir():
            return "# *** FAILED: getCmakeCache: no BUILD dir ***"

        bdir = os.path.join(rasync.COGX_ROOT, "BUILD")
        (result, errors) = procman.xrun_capture("../tools/scripts/cmake-freeze", workdir=bdir)

        s = "\n".join(result)

        return s

    def applyCmakeCache(self, cmakeCache, current=None):
        if not self._checkBuildDir():
            return

        bdir = os.path.join(rasync.COGX_ROOT, "BUILD")
        bfile = os.path.join(bdir, "_initcache.txt")
        f = open(bfile, "w")
        f.write(cmakeCache)
        f.close()

        procman.xrun_wait("../tools/scripts/cmake-apply . _initcache.txt", workdir=bdir)

    def startRsync(self, srcHost, current=None):
        p = self.manager.getProcess("RSYNC")
        if not p:
            return False

        if p.isRunning():
            return True

        RSYNC_DAEMON.srcHost = srcHost
        RSYNC_DAEMON.writeDaemonConfig()

        p.start(params = RSYNC_DAEMON.getDaemonParams())
        return True

    def stopRsync(self, current=None):
        p = self.manager.getProcess("RSYNC")
        if not p:
            return False

        p.stop()
        return True

    def _checkBuildDir(self):
        bdir = os.path.join(rasync.COGX_ROOT, "BUILD")
        bfile = os.path.join(bdir, "CMakeCache.txt")
        if os.path.exists(bfile):
            return True
        if not os.path.exists(bdir):
            try: os.mkdir(bdir)
            except: return False
        if not os.path.exists(bdir):
            return False
        procman.xrun_wait("cmake ..", workdir=bdir)

        return os.path.exists(bfile)

    def startBuild(self, target, current=None):
        p = self.manager.getProcess("BUILD")
        if not p:
            return False

        if p.isRunning():
            return True

        if not self._checkBuildDir():
            return False

        params = { "TARGET": "" if target == None else target }
        bdir = os.path.join(rasync.COGX_ROOT, "BUILD")
        p.start(params=params, workdir=bdir)
        return True

    def stopBuild(self, current=None):
        p = self.manager.getProcess("BUILD")
        if not p:
            return False

        p.stop()
        return True


class CCastSlave(threading.Thread):
    """
    CCastSlave runs an Agent servant in a separate thread. 
    An instance of CCastSlave can execute at most once (it's a Thread object).
    """
    def __init__(self, processManager, options, iceAddress= "tcp -p %d" % SLAVE_PORT):
        threading.Thread.__init__(self, name = "CastAgent")
        self.manager = processManager
        self.options = options
        self.address = iceAddress
        self._ic = None

    def run(self): # Thread.run
        # LOGGER.log("Starting CCastSlave")
        if self._ic != None:
            LOGGER.warn("... Warning: communicator already created.")
            return -1
        status = 0
        try:
            self._ic = Ice.initialize([]) # no sys.argv
            adapter = self._ic.createObjectAdapterWithEndpoints(self.name + "Adapter", self.address)
            obj = CAgentI(self.manager, self.options)
            adapter.add(obj, self._ic.stringToIdentity(self.name))
            adapter.activate()
            self._ic.waitForShutdown() # No event loop here, just waiting for shutdown()
        except:
            LOGGER.error(traceback.format_exc(20))
            status = 1
 
        # Clean up
        try:
            if self._ic != None: self._ic.destroy()
            self._ic = None
        except:
            LOGGER.error(traceback.format_exc(20))
            status = 1

        return status

    def shutdown(self):
        try:
            if self.manager != None: self.manager.stopAll()
        except:
            LOGGER.error(traceback.format_exc(20))
        try:
            if self._ic != None: self._ic.shutdown()
        except:
            LOGGER.error(traceback.format_exc(20))


def discoverRemoteHosts(hosts, tcpPort):
    """
    Tries to connect to every host in the list.
    Returns the list of discovered hosts that are running a Cast Agent
    on port tcpPort.
    """
    class _Checker(threading.Thread):
        def __init__(self, iceAddress):
            threading.Thread.__init__(self, name=iceAddress)
            self.iceAddr = iceAddress
            self.timeout = 5*500 # ice_timeout(seconds*500)
            self.success = False

        def run(self):
            self.success = False
            try:
                ic = Ice.initialize([])
                base = ic.stringToProxy(self.iceAddr).ice_timeout(self.timeout)
                remote = CastAgent.AgentPrx.checkedCast(base)
                if not remote: raise RuntimeError("Invalid proxy")
                procs = remote.getProcessList()
                self.success = True
                ic.destroy()
            except:
                try:
                    if ic != None: ic.destroy()
                except: pass

    checker = []
    for haddr in hosts:
        checker.append( (haddr, _Checker("CastAgent:tcp -h %s -p %d" % (haddr, tcpPort))) )

    for (haddr, chkr) in checker: chkr.start()
    for (haddr, chkr) in checker: chkr.join()

    working = []
    for (haddr, chkr) in checker:
        if chkr.success: working.append(haddr)

    return working
