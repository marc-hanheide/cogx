#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jan 2010 

import os, sys, time, re
import shutil
import optparse
from string import Template

from core import castagentsrv, procman, options, messages
# LOGGER = messages.CStdoutLogger()
LOGGER = messages.CInternalLogger()
procman.LOGGER = LOGGER
castagentsrv.LOGGER = LOGGER

import threading
import Ice
import core.modice
import icemodule.castcontrol.CastAgent as CastAgent

CODEF = '\x1b[0m'
BGK = '\x1b[1;40m'
BGY = '\x1b[1;43m'
FGR = '\x1b[1;31m'
FGB = '\x1b[1;34m'
FGW = '\x1b[1;37m'


class CLogDisplayer(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, name="CLogDisplayer")
        self.log = messages.CLogMerger()
        self.showFlush = True # option
        self.showWarning = True # option
        self.showError = True # option
        self.reError = re.compile(r"\b(error)\b", re.IGNORECASE)
        self.reWarning = re.compile(r"\b(warning)\b", re.IGNORECASE)
        self._isRunning = False
        self.out = sys.stdout
        # self.out = open("out.tmp", "w")

    def _markWords(self, text, coDefault=CODEF):
        text = self.reError.sub(FGW + BGY + r'\1' + coDefault, text)
        text = self.reWarning.sub(FGW + BGY + r'\1' + coDefault, text)
        return text

    def pullLogs(self):
        mods = False
        self.log.merge()
        if len(self.log.messages) < 1: pass
        else:
            msgs = self.log.getNewMessages(100)
            for m in msgs:
                text = m.getText().rstrip()
                co = None
                if m.msgtype != messages.CMessage.CASTLOG:
                    if m.msgtype == messages.CMessage.WARNING:
                        if not self.showWarning: continue
                        text = self._markWords(text, CODEF+FGB)
                        co = FGB
                    elif m.msgtype == messages.CMessage.ERROR:
                        if not self.showError: continue
                        text = self._markWords(text, CODEF+FGR)
                        co = FGR
                    elif m.msgtype == messages.CMessage.FLUSHMSG:
                        if not self.showFlush: continue
                        text = self._markWords(text, CODEF+FGW)
                        co = FGW
                if co != None: self.out.write(co)
                self.out.write(text)
                self.out.write(CODEF)
                self.out.write("\n")
            mods = True
        return mods

    def run(self):
        self._isRunning = True
        while self._isRunning:
            self.pullLogs()
            time.sleep(0.5)
        self.pullLogs()

    def shutdown(self):
        self._isRunning = False



class CConsoleAgent:
    def __init__(self, appOptions):
        port = appOptions.port
        self.manager = procman.CProcessManager("localhost")
        self._options = options.CCastOptions()
        self._options.loadConfig(appOptions.config)
        self._options.configEnvironment()
        self.address = "tcp -p %d" % port
        self.agent = None
        self._initLocalProcesses(appOptions)
        self._initMessagePump(appOptions)


    def _initLocalProcesses(self, appOptions):
        self.manager.addProcess(procman.CProcess("cast-java", self._options.xe("${CMD_JAVA_SERVER}")))
        self.manager.addProcess(procman.CProcess("cast-cpp", self._options.xe("${CMD_CPP_SERVER}")))
        self.manager.addProcess(procman.CProcess("cast-python", self._options.xe("${CMD_PYTHON_SERVER}")))
        #self.manager.addProcess(procman.CProcess("client", self._options.xe("${CMD_CAST_CLIENT}")))
        if appOptions.player_cfg != None:
            # TODO: Player configuration (file contents) could also be sent from the remote machine
            if not os.path.exists(appOptions.player_cfg):
                LOGGER.warn("Player configuration file '%s' not found." % appOptions.player_cfg)
            else:
                cmd = self._options.xe("${CMD_PLAYER}")
                cmd = cmd.replace("[PLAYER_CONFIG]", appOptions.player_cfg)
                self.manager.addProcess(procman.CProcess("player", cmd))
         if appOptions.golem_cfg != None:
            if not os.path.exists(appOptions.golem_cfg):
                LOGGER.warn("Golem configuration file '%s' not found." % appOptions.golem_cfg)
            else:
                cmd = self._options.xe("${CMD_GOLEM}")
                cmd = cmd.replace("[GOLEM_CONFIG]", appOptions.golem_cfg)
                self.manager.addProcess(procman.CProcess("golem", cmd))
        #self.manager.addProcess(procman.CProcess("peekabot", self._options.xe("${CMD_PEEKABOT}")))
        #self.procBuild = procman.CProcess("BUILD", 'make [target]', workdir=self._options.xe("${COGX_BUILD_DIR}"))
        #self.procBuild.allowTerminate = True
        #self.manager.addProcess(self.procBuild)


    def _initMessagePump(self, appOptions):
        self.mainLog = CLogDisplayer()
        for proc in self.manager.proclist:
            self.mainLog.log.addSource(proc)
        # XXX LOGGER not added if it's a CStdoutLogger - it will print it's own messages
        self.mainLog.log.addSource(LOGGER)
        self.mainLog.start()


    def _shutdown(self, agent):
        count = 10
        agent.shutdown()
        while agent.isAlive():
            LOGGER.log("... waiting for shutdown")
            time.sleep(1.0)
            count -= 1
            if count <= 0: break
        if agent.isAlive():
            LOGGER.warn("Server didn't shut down")
        else:
            LOGGER.log("Server stopped.")

    def startServing(self):
        if self.agent != None: self.stopServing()
        self.agent = castagentsrv.CCastSlave(self.manager, self._options, self.address)
        self.agent.start()
        time.sleep(0.2)

    def stopServing(self):
        if self.agent != None: self._shutdown(self.agent)
        self.agent = None
        self.manager.stopReaderThread()
        self.mainLog.shutdown()

def parseOptions():
    usage = "Usage: %prog [options] args"
    parser = optparse.OptionParser(usage)

    #parser.add_option("-v", "--verbose", action="store", type="int", dest="verbose")
    #parser.add_option("-q", "--quiet", action="store_const", const=0, dest="verbose")
    parser.add_option("-p", "--port", action="store", type="int", default=castagentsrv.SLAVE_PORT, dest="port",
        help="Set the port number on which this agent will be listening. Default=%d." % castagentsrv.SLAVE_PORT)
    parser.add_option("-c", "--config", action="store", type="string", default="castcontrol.conf", dest="config",
        help="Set a configuration file. Default=castcontrol.conf.")
    parser.add_option("", "--player", action="store", type="string", default=None, dest="player_cfg",
        help="Set the Player configuration file. If not set, Player won't be started by this agent.")
    parser.add_option("", "--golem", action="store", type="string", default=None, dest="golem_cfg",
        help="Set the Golem configuration file. If not set, Golem won't be started by this agent.")

    (options, args) = parser.parse_args()
    # if options.verbose > 3: print "Options parsed"
    # if len(args) != 1: parser.error("incorrect number of arguments")
    return (options, args)


def main():
    opts, args = parseOptions()
    print opts, args
    agent = CConsoleAgent(opts)
    agent.startServing()
    try:
        print "Press Ctrl-C to stop serving."
        while True: time.sleep(1.0)
    except KeyboardInterrupt:
        print "\nInterrupted\n"
        agent.stopServing()

if __name__ == "__main__": main()

