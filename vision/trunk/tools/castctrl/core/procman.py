#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009 

import os, sys, errno
import re
import time
import subprocess as subp
import select, signal
from collections import deque
import options, messages
import itertools


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
    DYING = 1
    STOPPED = 0
    TERM = -1
    ERROR = -2
    def  __init__(self, name, command, params=None, workdir=None):
        self.name = name
        self.command = command
        self.params = params   # Configurable parameters
        self.workdir = workdir
        self.process = None
        self.status = CProcess.STOPPED # 0 stoped; > 0 pid; < 0 error state; see getStatusStr()
        self.lastPollState = 0
        self.keepalive = False
        self.allowTerminate = False # CAST apps should not autoTerm; others may
        self.restarted = 0
        self.messages = deque() # 2.6 deque(maxlen=500)
        self.errors = deque() # 2.6 deque(maxlen=200)
        self.msgOrder = 0
        self.observers = []
        self.willClearAt = None

    def getStatusStr(self):
        if self.status == CProcess.DYING: return "Dying"
        if self.status == CProcess.TERM:  return "Terminated unexpectedly"
        if self.status == CProcess.ERROR: return "Internal error"
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
        if old != newStatus:
            for ob in self.observers: # TODO separate thread?
                ob.notifyStatusChange(self, old, newStatus)

    def start(self, params=None):
        if self.isRunning():
            warn("Process '%s' is already running" % self.name)
            return
        if params == None: params = self.params
        else: self.params = params
        if self.command == None:
            error("No command for process '%s'" % self.name)
            return
        command = self.command
        if params != None:
            for par in params.iterkeys():
                command = command.replace("[%s]" % par, params[par])
        command = command.split()
        log("Command '%s'" % " ".join(command))
        if self.workdir != None: log("PWD='%s'" % self.workdir)
        self.process = subp.Popen(command, bufsize=1, stdout=subp.PIPE, stderr=subp.PIPE, cwd=self.workdir)
        self._setStatus(max(2, self.process.pid))
        log("Process '%s' started, pid=%d" % (self.name, self.process.pid))
        time.sleep(0.01)

    def _clear(self, notify = True):
        self.restarted = 0
        self.process = None
        if not notify: self.status = CProcess.STOPPED
        else: self._setStatus(CProcess.STOPPED)

    def stop(self):
        if self.process == None:
            self._clear()
            return
        try:
            for sig in [signal.SIGQUIT, signal.SIGTERM, signal.SIGKILL]:
                try:
                    os.kill(self.process.pid, sig) # self.process.send_signal(sig) # Not in 2.5
                    time.sleep(0.05)
                except: pass
                tries = 100
                while self.isRunning() and tries > 0:
                    tries -= 1; time.sleep(0.02)
                if self.isRunning():
                    warn("Process '%s' did not respond to SIG -%d." % (self.name, sig))
        except Exception:
            # 2.6 except Exception as e:
            error("Failed to terminate process '%s' (%d)" % (self.name, self.process.pid))
            # error("Reason: %s", e)
        if not self.isRunning():
            log("Process '%s' stopped" % (self.name))
            self._clear()
        else: warn("Process '%s' is still running" % (self.name))

    def getPipes(self):
        pipes = []
        if self.process != None:
            if self.process.stdout != None: pipes.append(self.process.stdout)
            if self.process.stderr != None: pipes.append(self.process.stderr)
        return pipes

    def readPipes(self, pipes, maxcount=9999):
        # if not self.isRunning(): return 0
        if self.process == None: return 0
        nl = 0
        for pipe in pipes:
            if pipe == self.process.stdout:
                # select() is used because readline() is blocking
                while len(select.select([pipe], [], [], 0.0)[0]) > 0:
                    msg = pipe.readline();
                    if not self.isRunning() and msg == "":
                        nl += 1
                        if nl > 200: break
                        continue
                    nl += 1
                    mo = messages.reColorEscape.match(msg)
                    if mo != None:
                        typ = messages.CMessage.CASTLOG
                    else: typ = messages.CMessage.MESSAGE
                    self.msgOrder += 1
                    msg = msg.decode("utf-8", "replace")
                    self.messages.append(messages.CMessage(msg, typ, self.msgOrder))
                    if nl > maxcount: break
            elif pipe == self.process.stderr:
                while len(select.select([pipe], [], [], 0.0)[0]) > 0:
                    msg = pipe.readline()
                    if not self.isRunning() and msg == "":
                        nl += 1
                        if nl > 200: break
                        continue
                    self.msgOrder += 1
                    msg = msg.decode("utf-8", "replace")
                    self.errors.append(messages.CMessage(msg, messages.CMessage.ERROR, self.msgOrder))
                    nl += 1
                    if nl > maxcount: break
        return nl

    def check(self):
        if (self.status != self.STOPPED) != (self.process != None):
            error("Internal error: Process '%s' in invalid state" % (self.name))
            if self.isRunning(): self.status = max(2, self.process.pid)
            else:
                self.process = None
                self._setStatus(CProcess.ERROR)
            return
        if self.isRunning(): return
        if self.status > 0 or self.status == CProcess.DYING:
            if self.allowTerminate:
                if self.status != CProcess.DYING:
                    self._setStatus(CProcess.DYING)
                    self.willClearAt = time.time() + 3
                if time.time() > self.willClearAt: self._clear()
                return
            error("Process %s terminated unexpectedly, signal=%d" % (self.name, self.lastPollState))
            if self.keepalive:
                log("Restarting process %s" % self.name)
                self.start()
                self.restarted += 1
            else: self._setStatus(CProcess.TERM)

class CProcessManager(object):
    def __init__(self, name="localhost"):
        self.name = name   # maybe machine name
        self.proclist = [] # managed processes

    def __del__(self):
        self.stopAll()

    def addProcess(self, process):
        for p in self.proclist:
            if p.name == process.name:
                warn("Process '%s' is already registered" % (process.name))
                return
        self.proclist.append(process)

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

    def checkProcesses(self):
        for proc in self.proclist: proc.check()

    def communicate(self, timeout=0.01):
        procs = self.proclist
        pipes = []
        for proc in procs: pipes.extend(proc.getPipes())
        ready = select.select(pipes, [], [], 0.0)
        now = time.time(); tm = now; tmend = now + timeout
        while (len(ready[0]) > 0 and now >= tm and now < tmend):
            time.sleep(0)
            count = 100 # batch size
            while len(ready[0]) > 0 and count > 0 and now >= tm and now < tmend:
                for proc in procs:
                    nl = proc.readPipes(ready[0], count)
                    count -= nl
                for proc in procs: pipes.extend(proc.getPipes())
                ready = select.select(pipes, [], [], 0.0)
                now = time.time()
            now = time.time()

        return len(ready[0]) > 0 # True if there might be more lines to read

    def getStatus(self, procname):
        p = self.getProcess(procname)
        if p != None:
            if proc.process == None: return "0"
            return "%d" % proc.process.pid
        return "0"
 
