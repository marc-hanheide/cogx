#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009 
import sys, traceback, Ice
import threading, time

import modice
import icemodule.castcontrol.CastAgent as CastAgent
# import icemodule.castcontrol.ProcessInfo as ProcessInfo
from core import messages, options, procman

LOGGER = messages.CInternalLogger()

# Ice servant
class CAgentI(CastAgent.Agent):
    def __init__(self, processManager, options):
        self.manager = processManager
        self.options = options

    def getProcessList(self, current=None):
        # LOGGER.log("Retireveing process list")
        plist = self.manager.proclist
        procs = [CastAgent.ProcessInfo(name=p.name, status=p.status, error=p.error) for p in plist]
        return procs

    #def readMessages(self, processName, startTime, current=None):
    #    p = self.manager.getProcess(processName)
    #    if p == None:
    #        if processName == "LOGGER": p = LOGGER
    #        else: return []
    #    if self.logs.has_key(processName): log = self.logs[processName]
    #    else:
    #        log = messages.CLogMerger()
    #        log.addSource(p)
    #        self.logs[processName] = log
    #    if log == None: return []
    #    log.merge()
    #    msgs = [m for m in log.messages]
    #    res = [msg.getText() for msg in msgs]
    #    return res

    def startProcess(self, processName, current=None):
        # LOGGER.log("Starting %s" % processName)
        p = self.manager.getProcess(processName)
        if p != None: p.start()
        return 1 if p != None else 0

    def stopProcess(self, processName, current=None):
        # LOGGER.log("Stopping %s" % processName)
        p = self.manager.getProcess(processName)
        if p != None: p.stop()
        return 1 if p != None else 0

class CCastSlave(threading.Thread):
    """
    CCastSlave runs an Agent servant in a separate thread. 
    An instance of CCastSlave can execute at most once (it's a Thread object).
    """
    def __init__(self, processManager, options, iceAddress= "tcp -p 7832"):
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


