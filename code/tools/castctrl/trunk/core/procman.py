#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009 

import os, sys, errno
import re
import time
import subprocess as subp
import select, signal, threading
import options, messages
import itertools
import legacy

LOGGER = None
def log(msg):
    global LOGGER
    if LOGGER != None: LOGGER.log(msg)
    else: print msg

def warn(msg):
    global LOGGER
    if LOGGER != None: LOGGER.warn(msg)
    else: print "!!!", msg

def error(msg):
    global LOGGER
    if LOGGER != None: LOGGER.error(msg)
    else: print "!!!", msg

class CProcessObserver(object):
    def notifyStatusChange(self, process, old, new):
        pass

    def notifyMessagesReady(self, process):
        pass

class CProcess(object):
    STOPPED = 0     # noraml state, not running
    STARTING = 1    # starting
    FLUSH = -1      # flushing when terminated correctly
    OK = 0
    ERRTERM = -1    # terminated unexpectedly
    ERROR = -2      # Internal error
    ERRSTART = -3   # could not start
    def  __init__(self, name, command, params=None, workdir=None):
        self.name = name
        self.command = command
        self.params = params   # Configurable parameters
        self.workdir = workdir
        self.process = None
        self.status = CProcess.STOPPED # 0 stoped; > 0 pid; < 0 error state; see getStatusStr()
        self.error = CProcess.OK
        self.lastPollState = 0
        self.keepalive = False
        self.allowTerminate = False # CAST apps should not autoTerm; others may
        self.restarted = 0
        self.messages = legacy.deque(maxlen=500)
        self.errors = legacy.deque(maxlen=200)
        self.lastLinesEmpty = 0 # suppress consecutive empty lines
        self.msgOrder = 0
        self.observers = []
        self.willClearAt = None # Flushing
        self.pipeReader = None

    def __del__(self):
        self.stop()

    def getStatusStr(self):
        if self.status == CProcess.FLUSH: return "Flushing..."
        if self.error == CProcess.ERRTERM: return "Terminated unexpectedly"
        if self.error == CProcess.ERROR: return "Internal error"
        if self.error == CProcess.ERRSTART: return "Failed to start"
        if self.status == CProcess.STARTING: return "Starting..."
        if self.process == None: return "Not started"
        if self.restarted > 0: return "%d (r%d)" % (self.process.pid, self.restarted)
        return "%d" % self.process.pid

    # 0 normal; 1 running; 2 error
    def getStatusLevel(self):
        if self.error != CProcess.OK: return 2
        if self.status != CProcess.STOPPED: return 1
        return 0

    def isRunning(self):
        if self.process == None: return False
        self.lastPollState = self.process.poll()
        if self.lastPollState == None: return True
        return False

    def _setStatus(self, newStatus):
        old = self.status
        self.status = newStatus
        # if old != newStatus:
        for ob in self.observers: # TODO separate thread?
            ob.notifyStatusChange(self, old, newStatus)

    def _beginFlush(self):
        self.willClearAt = time.time() + 1
        self._setStatus(CProcess.FLUSH)

    def start(self, params=None):
        if self.isRunning():
            warn("Process '%s' is already running" % self.name)
            return
        if params == None: params = self.params
        else: self.params = params
        if self.command == None:
            error("No command for process '%s'" % self.name)
            return
        self.error = CProcess.OK
        command = self.command
        if params != None:
            for par in params.iterkeys():
                command = command.replace("[%s]" % par, params[par])
        command = command.split()
        log("CMD=%s" % " ".join(command))
        if self.workdir != None: log("PWD=%s" % self.workdir)
        try:
            self._setStatus(CProcess.STARTING)
            if self.pipeReader != None: self.pipeReader.stop()
            self.pipeReader = CPipeReader_1(self)
            self.pipeReader.start()
            self.process = subp.Popen(command, bufsize=1, stdout=subp.PIPE, stderr=subp.PIPE, cwd=self.workdir)
            time.sleep(0.01)
            self._setStatus(max(1, self.process.pid))
            log("Process '%s' started, pid=%d" % (self.name, self.process.pid))
        except Exception, e:
            self.process = None
            self.error = CProcess.ERRSTART
            self._setStatus(CProcess.STOPPED)
            error("Process '%s' failed to start" % (self.name))
            error("%s" % e)
        time.sleep(0.01)

    def _clear(self, notify = True):
        self.restarted = 0
        self.process = None
        nextStatus = CProcess.STOPPED
        if not notify: self.status = nextStatus
        else: self._setStatus(nextStatus)
        if self.pipeReader != None:
            self.pipeReader.stop()
            self.pipeReader = None

    def stop(self):
        self.error = CProcess.OK
        if self.process == None:
            self._clear()
            return
        try:
            # for sig in [signal.SIGQUIT, signal.SIGTERM, signal.SIGKILL]:
            for sig in [signal.SIGTERM, signal.SIGKILL]:
                try:
                    os.kill(self.process.pid, sig) # self.process.send_signal(sig) # Not in 2.5
                    time.sleep(0.05)
                except: pass
                tries = 100
                while self.isRunning() and tries > 0:
                    tries -= 1; time.sleep(0.03)
                if self.isRunning():
                    warn("Process '%s' did not respond to SIG -%d." % (self.name, sig))
        except Exception:
            # 2.6 except Exception as e:
            error("Failed to terminate process '%s' (%d)" % (self.name, self.process.pid))
            # error("Reason: %s", e)
        if not self.isRunning():
            log("Process '%s' stopped" % (self.name))
            self._beginFlush() # self._clear() # TODO: flush
        else: warn("Process '%s' is still running" % (self.name))

    def getPipes(self):
        pipes = []
        if self.process != None:
            if self.process.stdout != None: pipes.append(self.process.stdout)
            if self.process.stderr != None: pipes.append(self.process.stderr)
        return pipes

    def _readPipe(self, pipe, maxcount, targetList, fnType, flushing=False):
        start = time.time(); tmend = start + 0.1
        nl = 0
        if flushing: maxempty = 0
        else: maxempty = 1
        while len(select.select([pipe], [], [], 0.0)[0]) > 0:
            msg = pipe.readline()
            if len(msg) < 1 or msg.isspace(): self.lastLinesEmpty += 1
            else: self.lastLinesEmpty = 0
            if self.lastLinesEmpty <= maxempty:
                typ = fnType(msg)
                self.msgOrder += 1
                msg = msg.decode("utf-8", "replace")
                targetList.append(messages.CMessage(msg, typ, self.msgOrder))
                nl += 1
            if nl > maxcount: break
            now = time.time()
            if now < start or now > tmend: break
            if not flushing and not self.isRunning(): break
        return nl

    def _flushPipes(self, maxcount):
        pipes = self.getPipes()
        start = time.time(); tmend = start + 0.2
        nl = 0
        selected = select.select(pipes, [], [], 0.0)[0]
        def flushType(msg): return messages.CMessage.FLUSHMSG
        while len(selected) > 0:
            for pipe in selected:
                nl += self._readPipe(pipe, 200, self.errors, flushType, flushing=True)
            if nl > maxcount: break
            now = time.time()
            if now < start or now > tmend: break
            selected = select.select(pipes, [], [], 0.0)[0]
        return nl

    def readPipes(self, pipes, maxcount=9999):
        if not self.isRunning():
            self._flushPipes(maxcount)
            return 0
        nl = 0
        def stderrType(msg): return messages.CMessage.ERROR
        def stdoutType(msg):
            mo = messages.reColorEscape.match(msg)
            if mo != None: return messages.CMessage.CASTLOG
            else: return messages.CMessage.MESSAGE
        for pipe in pipes:
            nempty = 0
            if pipe == self.process.stdout:
                nl += self._readPipe(pipe, maxcount-nl, self.messages, stdoutType)
            elif pipe == self.process.stderr:
                nl +=  self._readPipe(pipe, maxcount-nl, self.errors, stderrType)
            if not self.isRunning(): break
        if not self.isRunning():
            nl += self._flushPipes(maxcount)
        return nl

    def check(self):
        if ((self.status != self.STOPPED) != (self.process != None)) and self.status != self.STARTING:
            error("Internal error: Process '%s' in invalid state" % (self.name))
            if self.isRunning(): self.status = max(1, self.process.pid)
            else:
                self.process = None
                self.error = CProcess.ERROR
                self._setStatus(CProcess.STOPPED)
            return
        if self.status == self.STOPPED or self.isRunning(): return
        if self.status == CProcess.FLUSH:
            if time.time() > self.willClearAt: self._clear()
        if self.status > 0:
            self._beginFlush()
            if self.allowTerminate: return

            error("Process '%s' terminated unexpectedly, signal=%d" % (self.name, self.lastPollState))
            if self.keepalive:
                log("Restarting process '%s'" % self.name)
                self.start()
                self.restarted += 1
            else:
                self.error = CProcess.ERRTERM
                self._beginFlush()

class CPipeReader_1(threading.Thread):
    def __init__(self, process):
        threading.Thread.__init__(self, name="Pipe Reader - %s" % process.name)
        self.process = process
        self.isRunning = False

    def run(self):
        self.isRunning = True
        while self.isRunning:
            pipes = []
            pipes.extend(self.process.getPipes())
            if len(pipes) < 1: time.sleep(0.02)
            else:
                (rlist, wlist, xlist) = select.select(pipes, [], [], 0.1)
                if len(rlist) > 0:
                    nl = self.process.readPipes(rlist, 200)

    def stop(self):
        self.isRunning = False

class CPipeReader(threading.Thread):
    def __init__(self, procManager):
        threading.Thread.__init__(self, name="Pipe Reader")
        self.procManager = procManager
        self.isRunning = False

    def run(self):
        self.isRunning = True
        while self.isRunning:
            procs = [p for p in self.procManager.proclist]
            pipes = []
            for proc in procs: pipes.extend(proc.getPipes())
            if len(pipes) < 1:
                time.sleep(0.02)
                continue
            (rlist, wlist, xlist) = select.select(pipes, [], [], 0.02)
            if len(rlist) > 0:
                now = time.time(); tm = now; tmend = now + 0.1
                for proc in procs: nl = proc.readPipes(rlist, 200)

    def stop(self):
        self.isRunning = False

class CProcessChecker(threading.Thread):
    def __init__(self, procManager):
        # TODO: Multiple process managers, different hosts!
        threading.Thread.__init__(self, name="Process Checker")
        self.procManager = procManager
        self.isRunning = False

    def run(self):
        self.isRunning = True
        while self.isRunning:
            for p in self.procManager.proclist: p.check()
            time.sleep(0.1)

    def stop(self):
        self.isRunning = False

class CProcessManager(object):
    def __init__(self, name="localhost"):
        self.name = name   # maybe machine name
        self.proclist = [] # managed processes
        self.pipeReaderThread = None
        self.procCheckerThread = CProcessChecker(self) # TODO: move to main or make global!
        self.procCheckerThread.start()

    def __del__(self):
        self.stopAll()
        self.stopReaderThread()

    def addProcess(self, process):
        for p in self.proclist:
            if p.name == process.name:
                warn("Process '%s' is already registered" % (process.name))
                return
        self.proclist.append(process)
        #if self.pipeReaderThread == None:
        #    self.pipeReaderThread = CPipeReader(self)
        #    self.pipeReaderThread.start()

    def getProcess(self, name):
        for p in self.proclist:
            if p.name == name: return p
        return None

    def removeProcess(self, process):
        if type(process) == type(""): process = self.getProcess(process)
        if process != None: self.proclist.remove(process)

    def removeAll(self, stop=True):
        if stop: self.stopAll()
        self.proclist = []

    def stopAll(self):
        for proc in self.proclist: proc.stop()

    #def checkProcesses(self): # MOVED to a separate thread
    #    for proc in self.proclist: proc.check()

    def stopReaderThread(self):
        if self.pipeReaderThread != None: self.pipeReaderThread.stop()
        self.pipeReaderThread = None
        if self.procCheckerThread != None: self.procCheckerThread.stop()
        self.procCheckerThread = None

    # MOVED to a separate thread!
    #def communicate(self, timeout=0.01):
    #    return 0
    #    procs = self.proclist
    #    pipes = []
    #    for proc in procs: pipes.extend(proc.getPipes())
    #    ready = select.select(pipes, [], [], 0.0)
    #    now = time.time(); tm = now; tmend = now + timeout
    #    while (len(ready[0]) > 0 and now >= tm and now < tmend):
    #        time.sleep(0)
    #        count = 100 # batch size
    #        while len(ready[0]) > 0 and count > 0 and now >= tm and now < tmend:
    #            for proc in procs:
    #                nl = proc.readPipes(ready[0], count)
    #                count -= nl
    #            for proc in procs: pipes.extend(proc.getPipes())
    #            ready = select.select(pipes, [], [], 0.0)
    #            now = time.time()
    #        now = time.time()

    #    return len(ready[0]) > 0 # True if there might be more lines to read
    
    def getStatus(self, procname):
        p = self.getProcess(procname)
        if p != None:
            if proc.process == None: return "0"
            return "%d" % proc.process.pid
        return "0"

    # Remote hosts need to be locked for exclusive use before any process is started
    def lockHost(self):
        return True
 
def runCommand(cmd, params=None, workdir=None, name="onetime"):
    try:
        p = CProcess(name, cmd, params=params, workdir=workdir)
        p.start()
        pipes = p.getPipes()
        while p.isRunning():
            ready = select.select(pipes, [], [], 0.1)
            if len(ready[0]) > 0: p.readPipes(ready[0], 100)
        ready = select.select(pipes, [], [], 0.0)
        if len(ready[0]) > 0: p.readPipes(ready[0], 100)
        if LOGGER != None:
            log = messages.CLogMerger()
            try: # TODO: these messages should be internal!
                log.addSource(p)
                log.merge()
                msgs = [m for m in log.messages]
                for m in msgs: LOGGER.addMessage(m)
            finally: log.removeSource(p)
    except Exception, e:
        error("Internal error")
        error("%s" % e)


