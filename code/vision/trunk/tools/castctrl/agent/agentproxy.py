#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jul 2009 

import sys, time, traceback, Ice
import modice
import icemodule.castcontrol.CastAgent as CastAgent
from core import messages, options, procman

class CRemoteProcess(object):
    def __init__(self, name, processManager):
        self.manager = processManager # forward requests to this manager
        self.name = name
    pass

class CRemoteProcessManager(CProcessManager):
    def __init__(self, machine="localhost", port=9999):
        CProcessManager.__init__(self, machine)
        self.agentProxy = # TODO

    def getStatus(self, procname):
        return "0"

    def checkProcesses(self):
        pass

    def communicate(self):
        return 0

