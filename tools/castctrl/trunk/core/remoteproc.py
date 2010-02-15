#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
import sys, traceback, Ice
import threading, time
from procman import CProcessBase, CRemoteHostInfo
from messages import CMessageSource, CMessage
import itertools
import legacy

import modice
import icemodule.castcontrol.CastAgent as CastAgent

class CRemoteProcess(CProcessBase):
    """
    A Remote Process accessed through a CASTControl Agent.
    It implements the same (public) interface as CProcess.
    It has only one message queue 'messages' so it doesn't work as a CMessageSource;
    messages are therefore merged in the manager which provides the interface
    for CLogMerger.
    """
    def __init__(self, manager, name, host):
        CProcessBase.__init__(self, name, host)
        self.manager = manager
        self.messages = legacy.deque(maxlen=500)

    def getStatusStr(self):
        st = CProcessBase.getStatusStr(self)
        if st != None: return st
        if self.status > 2: return "%d" % (self.status)
        return "Not started"

    def start(self):
        self.manager.agentProxy.startProcess(self.name)

    def stop(self):
        self.manager.agentProxy.stopProcess(self.name)

    def _readMessages(self):
        msgs = self.manager.agentProxy.readMessages(self.name)
        for m in msgs:
            self.messages.append(CMessage("R:"+m.message, m.msgtype, timestamp=m.time))


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
        for p in self.proclist: p._readMessages()
        msgs = self.agentProxy.readMessages("LOGGER")
        for m in msgs:
            self.remoteInternalMessages.append(CMessage("RI:" + m.message, m.msgtype, timestamp=m.time))


    def stopAll(self):
        # self.agentProxy.stopAll()
        pass


class CRemoteMessageSource(CMessageSource):
    def __init__(self, remoteManager):
        CMessageSource.__init__(self, remoteManager)
        self.yieldErrors = False

    # FIXME: Iterators crash if the queue is changed from another thread.
    # Instead of iterators, this function should probably return a list of filtered messages.
    def getIterators(self):
        its = []
        if self.tmLastSeen < self.tmLastPop: self.tmLastSeen = self.tmLastPop
        def checkTime(msg):
            if msg.time > self.tmLastSeen:
                self.tmLastPop = msg.time
                return True
            return False
        if self.yieldMessages:
            # NOTE: self.process is a remoteManager
            for proc in self.process.proclist:
                its.append(itertools.ifilter(lambda x: checkTime(x), proc.messages))
            its.append(itertools.ifilter(lambda x: checkTime(x), self.process.remoteInternalMessages))
        return its

