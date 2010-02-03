#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
import sys, traceback, Ice
import threading, time
from procman import CProcessBase, CRemoteHostInfo

import modice
import icemodule.castcontrol.CastAgent as CastAgent

# TODO: use paramiko or fabric
#class CRemoteProcessManager(CProcessManager):
#    def __init__(self, name):
#        CProcessManager.__init__(self, name)
#        # defaults; TODO: read from castcrtl.conf or ~/.castctrl/hosts.conf ... [server-<name>]
#        self.login = "-i /home/user/.ssh/cogx_rsa mmarko@localhost"
#        self.remoteCmd = "ssh [login] [command]"
#        self.remoteCmdX = "ssh -Y [login] [command]"

#    def composeRemoteCommand(self, command, xserver=False):
#        if xserver: cmd = self.remoteCmdX
#        else: cmd = self.remoteCmd
#        cmd = cmd.replace("[login]", self.login)
#        cmd = cmd.replace("[command]", command)

#    def lockHost(self):
#        return False # TODO
#

class CRemoteProcess(CProcessBase):
    """
    A Remote Process accessed through a CASTControl Agent.
    It implements the same (public) interface as CProcess.
    """
    def __init__(self, manager, name, host):
        CProcessBase.__init__(self, name, host)
        self.manager = manager

    def getStatusStr(self):
        st = CProcessBase.getStatusStr(self)
        if st != None: return st
        if self.status > 2: return "%d" % (self.status)
        return "Not started"

    def start(self):
        self.manager.agentProxy.startProcess(self.name)

    def stop(self):
        self.manager.agentProxy.stopProcess(self.name)

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

        rnew = []
        # Update and add new processes
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

        # remove inexistent processes (an unlikely event)
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

    def stopAll(self):
        # self.agentProxy.stopAll()
        pass

