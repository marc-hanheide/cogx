#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jan 2010 

import sys, traceback, Ice
import threading, time
from procman import CProcessBase, CRemoteHostInfo
from messages import CLogMessageSource, CMessage
import itertools
import legacy

import modice
import icemodule.castcontrol.CastAgent as CastAgent

class CRemoteProcess(CProcessBase):
    """
    A Remote Process accessed through a CASTControl Agent.
    """
    def __init__(self, manager, name, host):
        CProcessBase.__init__(self, name, host)
        self.manager = manager
        self.messages = legacy.deque(maxlen=500)
        self.srcid = "remote.%s.%s" % (host.host.replace('.', '_'), name.replace('.', '_'))
        self._lock = threading.Lock()

    def getMessages(self):
        try:
            self._lock.acquire(True)
            msgs = list(self.messages)
            self.messages.clear()
        finally:
            self._lock.release()
        return msgs

    def getStatusStr(self):
        st = CProcessBase.getStatusStr(self)
        if st != None: return st
        if self.status > 2: return "%d" % (self.status)
        return "Not started"

    def start(self):
        try:
            self.manager.agentProxy.startProcess(self.name)
        except Ice.ConnectionRefusedException:
            pass
        except Ice.TimeOutException:
            print "Remote process", slef.name, "start() timed out"

    def stop(self):
        try:
            self.manager.agentProxy.stopProcess(self.name)
        except Ice.ConnectionRefusedException:
            pass
        except Ice.TimeOutException:
            print "Remote process", slef.name, "stop() timed out"

    def _readMessages(self):
        msgs = self.manager.agentProxy.readMessages(self.name)
        msgs = [CMessage(self.srcid, "R:"+m.message, m.msgtype, timestamp=m.time) for m in msgs]
        try:
            self._lock.acquire(True)
            for m in msgs: self.messages.append(m)
        finally:
            self._lock.release()


class CRemoteProcessManager:
    """
    A Remote Process Manager communicates with a CASTControl Agent.
    It implements the same (public) interface as CProcessManager.
    """
    def __init__(self, name, address, port):
        self.name = name
        self.address = address
        self.port = port
        self.proclist = []
        self._ic = None
        self._agentProxy = None
        self.online = False
        self.observers = [] # proclist change, etc.
        self.remoteInternalMessages = legacy.deque(maxlen=500)
        self.srcid = "remote.%s.%s" % (self.name.replace('.', '_'), self.address.replace('.', '_'))
        self._lock = threading.Lock()

    def getRemoteInternalMessages(self):
        try:
            self._lock.acquire(True)
            msgs = list(self.remoteInternalMessages)
            self.remoteInternalMessages.clear()
        finally:
            self._lock.release()
        return msgs

    def getStatusStr(self): # For processtree
        if self.online: return self.address
        else: return "OFFLINE"

    def getStatusLevel(self): # For processtree
        if self.online: return 0 # Normal
        else: return 2 # Error

    @property
    def agentProxy(self):
        if self._agentProxy != None: return self._agentProxy
        if self._ic != None: return None # Errors happened in previous initialization

        iceaddr = "CastAgent:tcp -h %s -p %d" % (self.address, self.port)
        self._ic = Ice.initialize([])
        base = self._ic.stringToProxy(iceaddr).ice_timeout(5*500)
        remote = CastAgent.AgentPrx.checkedCast(base)
        if not remote: raise RuntimeError("Invalid proxy")
        self._agentProxy = remote
        return self._agentProxy

    def updateProcessList(self):
        try:
            procs = self.agentProxy.getProcessList()
            self.online = True
        except:
            self.online = False
            # TODO: notify process-manager observers
            return

        # remote-new; Update and add new processes
        rnew = []
        for pr in procs:
            found = False
            for pl in self.proclist:
                if pl.name == pr.name:
                    old = pl.status
                    pl.status = pr.status
                    pl.error = pr.error
                    pl.notifyStatusChange(old, pl.status)
                    found = True

            if not found:
                np = CRemoteProcess(self, pr.name, CRemoteHostInfo(self.name))
                np.status = pr.status
                np.error = pr.error
                rnew.append(np)

        # local-old; remove inexistent processes (an unlikely event)
        lold = []
        for pl in self.proclist:
            found = False
            for pr in procs:
                if pr.name == pl.name:
                    found = True
                    break
            if not found: lold.append(pl)

        # update the list
        for pl in lold: self.proclist.remove(pl)
        self.proclist += rnew

        # TODO: if lold or rnew nonempty, notify process-manager observers


    def getProcess(self, name):
        for p in self.proclist:
            if p.name == name: return p
        return None


    def pumpRemoteMessages(self):
        try:
            for p in self.proclist: p._readMessages()
            msgs = self.agentProxy.readMessages("LOGGER")
            msgs = [CMessage(self.srcid, "RI:"+m.message, m.msgtype, timestamp=m.time) for m in msgs]
            try:
                self._lock.acquire(True)
                for m in msgs: self.remoteInternalMessages.append(m)
            finally:
                self._lock.release()
            self.online = True
        except Ice.ConnectionRefusedException:
            self.online = False

    def stopAll(self):
        # self.agentProxy.stopAll()
        pass


#class CRemoteMessageSource(CMessageSource):
#    def __init__(self, remoteManager):
#        CMessageSource.__init__(self, remoteManager)
#        self.yieldErrors = False

#    def getMessages(self, clear=True):
#        msgs = []
#        if self.yieldMessages:
#            for proc in self.process.proclist:
#                msgs += proc.getMessages(clear)
#            msgs += self.process.getRemoteInternalMessages(clear)
#        return msgs

# CRemoteMessageSource merges messages from a remote host.
class CRemoteLogMessageSource(CLogMessageSource):
    def __init__(self, remoteManager):
        self.remoteManager = remoteManager

    def getLogMessages(self): # CLogMessageSource
        msgs = []
        for proc in self.remoteManager.proclist:
            msgs += proc.getMessages()
        msgs += self.remoteManager.getRemoteInternalMessages()
        return msgs

    def isSameLogMessageSource(self, aObject): #CLogMessageSource
        return aObject == self

