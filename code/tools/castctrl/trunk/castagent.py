#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim

import os, sys, time, re
import shutil
import optparse
from string import Template

from core import castagent, procman, options, messages
# LOGGER = messages.CStdoutLogger()
LOGGER = messages.CInternalLogger()
procman.LOGGER = LOGGER
castagent.LOGGER = LOGGER

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
        self._initLocalProcesses()
        self._initMessagePump()


    def _initLocalProcesses(self):
        self.manager.addProcess(procman.CProcess("server-java", options.xe("${CMD_JAVA_SERVER}")))
        self.manager.addProcess(procman.CProcess("server-cpp", options.xe("${CMD_CPP_SERVER}")))
        self.manager.addProcess(procman.CProcess("server-python", options.xe("${CMD_PYTHON_SERVER}")))
        #self.manager.addProcess(procman.CProcess("client", options.xe("${CMD_CAST_CLIENT}")))
        #self.manager.addProcess(procman.CProcess("player", options.xe("${CMD_PLAYER}")))
        #self.manager.addProcess(procman.CProcess("peekabot", options.xe("${CMD_PEEKABOT}")))
        #self.procBuild = procman.CProcess("BUILD", 'make [target]', workdir=options.xe("${COGX_BUILD_DIR}"))
        #self.procBuild.allowTerminate = True
        #self.manager.addProcess(self.procBuild)


    def _initMessagePump(self):
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
        self.agent = castagent.CCastSlave(self.manager, self._options, self.address)
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

    parser.add_option("-v", "--verbose", action="store", type="int", dest="verbose")
    parser.add_option("-q", "--quiet", action="store_const", const=0, dest="verbose")
    parser.add_option("-p", "--port", action="store", type="int", default=7832, dest="port")
    parser.add_option("-c", "--config", action="store", type="string", default="castcontrol.conf", dest="config")

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

