#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009

import os, sys, errno
import re
import time
import subprocess as subp
import tempfile
import select, signal, threading
import fcntl
import options, messages, logger
import itertools
import legacy

LOG4J_PROCESS = "log4jServer"

LOGGER = logger.get()
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

def cmdlineToArray(cmd):
    cmd = cmd.split()
    res = []
    icmd = iter(cmd)
    for p in icmd:
        cp = p
        if p.startswith('"') and not p.endswith('"'):
            for p in icmd:
                cp = cp + " " + p
                if p.endswith('"'): break
        elif p.startswith("'") and not p.endswith("'"):
            for p in icmd:
                cp = cp + " " + p
                if p.endswith("'"): break

        if (cp.startswith('"') and cp.endswith('"')) or (cp.startswith("'") and cp.endswith("'")):
            cp = cp[1:-1]

        res.append(cp)

    return res

class CLambdaThread(threading.Thread):
    def __init__(self, data, func):
        threading.Thread.__init__(self)
        self.data = data
        self.func = func
    def run(self):
        self.func(self.data)

class CProcessObserver(object):
    def notifyStatusChange(self, process, old, new):
        pass

    def notifyMessagesReady(self, process):
        pass

# Remote process execution
class CRemoteHostInfo(object):
    def __init__(self, host = "localhost"):
        self.host = host
        if self.host == None: self.host = "?"
        self.cmdPrefix = ""

class CProcessBase(object):
    STOPPED = 0     # noraml state, not running
    STARTING = 1    # starting
    STOPPING = 2    # stopping
    FLUSH = -1      # flushing when terminated correctly
    OK = 0
    ERRTERM = -1    # terminated unexpectedly
    ERROR = -2      # Internal error
    ERRSTART = -3   # could not start
    def __init__(self, name, host=None):
        self.host = host
        self.name = name
        self.status = CProcessBase.STOPPED # 0 stoped; > 0 pid; < 0 error state; see getStatusStr()
        self.error = CProcessBase.OK
        self.observers = []
        self.killSignals = [signal.SIGTERM]
        if self.name == None: self.name = "?"
        pass

    def getStatusStr(self):
        if self.status == CProcessBase.FLUSH: return "Flushing..."
        if self.error == CProcessBase.ERRTERM: return "Terminated unexpectedly"
        if self.error == CProcessBase.ERROR: return "Internal error"
        if self.error == CProcessBase.ERRSTART: return "Failed to start"
        if self.status == CProcessBase.STARTING: return "Starting..."
        if self.status == CProcessBase.STOPPING: return "Stopping..."
        return None

    # 0 normal; 1 running; 2 error
    def getStatusLevel(self):
        if self.error != CProcessBase.OK: return 2
        if self.status != CProcessBase.STOPPED: return 1
        return 0

    def start(self):
        pass

    def stop(self):
        pass

    def notifyStatusChange(self, oldStatus, newStatus):
        for ob in self.observers: # TODO separate thread?
            ob.notifyStatusChange(self, oldStatus, newStatus)

class CProcess(CProcessBase):
    def  __init__(self, name, command, params=None, workdir=None, host=None, allowTerminate=False):
        if host == None: host = CRemoteHostInfo()
        CProcessBase.__init__(self, name, host)
        self.command = command
        self.params = params   # Configurable parameters
        self.workdir = workdir
        self.process = None
        self.lastPollState = 0
        self.keepalive = False
        self.allowTerminate = allowTerminate # CAST apps should not autoTerm; others may
        self.restarted = 0
        self.messages = legacy.deque(maxlen=500)
        self.messageProcessor = None  # Some servers (log4j) could do additional message processing
        self.errors = legacy.deque(maxlen=200)
        self.lastLinesEmpty = 0 # suppress consecutive empty lines
        self.msgOrder = 0
        self.willClearAt = None # Flushing
        self.pipeReader = None
        self.srcid = "process.%s.%s" % (self.host.host.replace('.', '_'), self.name.replace('.', '_'))
        self.partialLine = {} # one place for every pipe

    def __del__(self):
        self.stop()

    def getMessages(self, clear=True):
        msgs = list(self.messages)
        if clear: self.messages.clear()
        return msgs

    def getErrors(self, clear=True):
        msgs = list(self.errors)
        if clear: self.errors.clear()
        return msgs

    def getStatusStr(self):
        st = CProcessBase.getStatusStr(self)
        if st != None: return st
        if self.process == None: return "Not started"
        if self.restarted > 0: return "%d (r%d)" % (self.process.pid, self.restarted)
        return "%d" % self.process.pid

    def isRunning(self):
        if self.process == None: return False
        self.lastPollState = self.process.poll()
        if self.lastPollState == None: return True
        return False

    def _setStatus(self, newStatus):
        old = self.status
        self.status = newStatus
        self.notifyStatusChange(old, newStatus)

    def _beginFlush(self):
        self.willClearAt = time.time() + 1
        self._setStatus(CProcessBase.FLUSH)

    def start(self, command=None, params=None, workdir=None, env=None, allowTerminate=None):
        if self.isRunning():
            warn("Process '%s' is already running" % self.name)
            return
        if params == None: params = self.params
        else: self.params = params

        if command == None or command.strip() == "":
            if self.command == None or self.command.strip() == "":
                error("No command for process '%s'" % self.name)
                return
            command = self.command

        self.error = CProcessBase.OK
        if params != None:
            for par in params.iterkeys():
                command = command.replace("[%s]" % par, "%s" % params[par])
        log("CMD=%s" % command)

        command = cmdlineToArray(command)
        if workdir == None or workdir.strip() == "":
            workdir = self.workdir
        if workdir != None: log("PWD=%s" % workdir)

        if allowTerminate != None:
            self.allowTerminate = True if allowTerminate else False

        try:
            self._setStatus(CProcessBase.STARTING)
            if self.pipeReader != None:
                self._stopReader()
            self.pipeReader = CPipeReader_1(self)
            self.pipeReader.start()
            self.process = subp.Popen(
                    command, bufsize=1,
                    stdout=subp.PIPE, stderr=subp.PIPE,
                    cwd=workdir, env=env)

            # Make the pipes nonblocking so we can read the messages better
            fcntl.fcntl(self.process.stdout, fcntl.F_SETFL, os.O_NONBLOCK)
            fcntl.fcntl(self.process.stderr, fcntl.F_SETFL, os.O_NONBLOCK)

            time.sleep(0.01)
            self._setStatus(max(1, self.process.pid))
            log("Process '%s' started, pid=%d" % (self.name, self.process.pid))
        except Exception, e:
            self.process = None
            self.error = CProcessBase.ERRSTART
            self._setStatus(CProcessBase.STOPPED)
            error("Process '%s' failed to start" % (self.name))
            error("Command: '%s'" % (" ".join(command)))
            error("%s" % e)
        time.sleep(0.01)

    def _clear(self, notify = True):
        self.restarted = 0
        self.process = None
        nextStatus = CProcessBase.STOPPED
        if not notify: self.status = nextStatus
        else: self._setStatus(nextStatus)
        self._stopReader()

    def _stopReader(self):
        if self.pipeReader != None:
            self.pipeReader.stop()
            self.pipeReader.join(0.2)
            if self.pipeReader and self.pipeReader.isAlive():
                error("Failed to join thread (Reader, %s)" % self.name)
            self.pipeReader = None

    def stop(self):
        self.error = CProcessBase.OK
        if self.process == None:
            self._clear()
            return
        log("Stopping process %s" % self.name)
        self._setStatus(CProcessBase.STOPPING)
        try:
            sigList = [(s, 100) for s in self.killSignals] + [(signal.SIGKILL, 20)]
            for sig,tries in sigList:
                try:
                    os.kill(self.process.pid, sig) # self.process.send_signal(sig) # Not in 2.5
                    time.sleep(0.2)
                except: pass
                while self.isRunning() and tries > 0:
                    tries -= 1
                    time.sleep(0.1)
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
        if not self.partialLine.has_key(pipe): self.partialLine[pipe] = ""
        nl = 0
        if flushing: maxempty = 0
        else: maxempty = 1
        while len(select.select([pipe], [], [], 0.0)[0]) > 0:
            msg = pipe.read(64000)
            if len(msg) > 0:
                lineTerminated = msg.endswith("\n")
                lines = msg.split("\n")
                if lineTerminated:
                    lines = lines[:-1]
                if (len(lines) > 1 or lineTerminated) and len(self.partialLine[pipe]) > 0:
                    lines[0] = self.partialLine[pipe] + lines[0]
                    self.partialLine[pipe] = ""
                if not lineTerminated:
                    self.partialLine[pipe] += lines[-1]
                    lines = lines[:-1]
                for msg in lines:
                    if len(msg) < 1 or msg.isspace(): self.lastLinesEmpty += 1
                    else: self.lastLinesEmpty = 0
                    if self.lastLinesEmpty > maxempty: continue
                    typ = fnType(msg)
                    if self.messageProcessor:
                        (msg, typ) = self.messageProcessor.process(msg, typ)
                    self.msgOrder += 1
                    msg = msg.decode("utf-8", "replace")
                    targetList.append(messages.CMessage(self.srcid, msg, typ, self.msgOrder))
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
        if self.status == self.STARTING or self.status == self.STOPPING: return
        if ((self.status == self.STOPPED) != (self.process == None)):
            error("Internal error: Process '%s' in invalid state" % (self.name))
            if self.isRunning(): self.status = max(1, self.process.pid)
            else:
                self.process = None
                self.error = CProcessBase.ERROR
                self._setStatus(CProcessBase.STOPPED)
            return
        if self.status == self.STOPPED or self.isRunning(): return
        if self.status == CProcessBase.FLUSH:
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
                self.error = CProcessBase.ERRTERM
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
            if len(pipes) < 1: time.sleep(0.3)
            else:
                (rlist, wlist, xlist) = select.select(pipes, [], [], 0.3)
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
                time.sleep(0.3)
                continue
            (rlist, wlist, xlist) = select.select(pipes, [], [], 0.3)
            if len(rlist) > 0:
                now = time.time(); tm = now; tmend = now + 0.2
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
        self.name = name    # maybe machine name
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

    def getStatusStr(self): # For processtree
        return ""

    def getStatusLevel(self): # For processtree
        return 0

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
        ts = []
        for p in self.proclist:
            t = CLambdaThread(p, lambda x: x.stop())
            ts.append(t)
            t.start()

        for t in ts: t.join()


    def stopReaderThread(self):
        #if self.pipeReaderThread != None:
        #    self.pipeReaderThread.stop()
        #    self.pipeReaderThread.join(2.0)
        #    if self.pipeReaderThread.isAlive():
        #        error("Failed to join thread (Reader)")
        #self.pipeReaderThread = None

        for proc in self.proclist: proc._stopReader()

        if self.procCheckerThread != None:
            self.procCheckerThread.stop()
            self.procCheckerThread.join(2.0)
            if self.procCheckerThread.isAlive():
                error("Failed to join thread (Checker)")
        self.procCheckerThread = None


    def getStatus(self, procname):
        p = self.getProcess(procname)
        if p != None:
            if proc.process == None: return "0"
            return "%d" % proc.process.pid
        return "0"

    # Remote hosts need to be locked for exclusive use before any process is started
    def lockHost(self):
        return True

# BAD: causes castcontrol to go to 100% of CPU use
#def runCommand(cmd, params=None, workdir=None, name="onetime"):
#    try:
#        p = CProcess(name, cmd, params=params, workdir=workdir)
#        p.start()
#        pipes = p.getPipes()
#        while p.isRunning():
#            ready = select.select(pipes, [], [], 0.1)
#            if len(ready[0]) > 0: p.readPipes(ready[0], 100)
#        ready = select.select(pipes, [], [], 0.0)
#        if len(ready[0]) > 0: p.readPipes(ready[0], 100)
#        if LOGGER != None:
#            log = messages.CLogMerger()
#            try: # TODO: these messages should be internal!
#                log.addSource(p)
#                log.merge()
#                msgs = [m for m in log.messages]
#                for m in msgs: LOGGER.addMessage(m)
#            finally: log.removeSource(p)
#    except Exception, e:
#        error("Internal error")
#        error("%s" % e)
#    finally:
#        if p != None: p.stop()

def xrun(cmdline, workdir=None, env=None):
    # XRUN may create zombies.
    cmds = cmdlineToArray(cmdline)
    cwd = os.getcwd()
    pid = None
    try:
        pid = subp.Popen(cmds, cwd=workdir, env=env).pid
        log("CMD pid=%d: %s" % (pid, cmdline))
    except Exception, e:
        error("xrun Internal: %s" % e)
    finally:
        os.chdir(cwd)
    log("")
    return pid

def xrun_wait(cmdline, workdir=None):
    # XRUN may create zombies.
    cmds = cmdlineToArray(cmdline)
    log("CMD: %s" % (cmdline))
    cwd = os.getcwd()
    try:
        std = tempfile.TemporaryFile()
        err = tempfile.TemporaryFile()
        if workdir != None: os.chdir(workdir)
        rv = subp.call(cmds, stdout=std, stderr=err)
        std.seek(0)
        for ln in std.readlines(): log(ln)
        std.close()
        err.seek(0)
        for ln in err.readlines(): error(ln)
        err.close()
    except Exception, e:
        error("xrun_wait Internal: %s" % e)
    finally:
        os.chdir(cwd)
    log("")

def xrun_capture(cmdline, workdir=None):
    # XRUN may create zombies.
    cmds = cmdlineToArray(cmdline)
    log("CMD: %s" % (cmdline))
    cwd = os.getcwd()
    result = None
    errors = None
    try:
        std = tempfile.TemporaryFile()
        err = tempfile.TemporaryFile()
        if workdir != None: os.chdir(workdir)
        rv = subp.call(cmds, stdout=std, stderr=err)
        std.seek(0)
        result = std.readlines()
        std.close()
        err.seek(0)
        errors = err.readlines()
        err.close()
        for ln in errors: error(ln)
    except Exception, e:
        error("xrun_capture Internal: %s" % e)
    finally:
        os.chdir(cwd)
    return (result, errors)

