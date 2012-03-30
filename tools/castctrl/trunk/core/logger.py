#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jan 2010 
#
# Internal logging for CAST Control.
#
# The logger has a "message source" intreface that is used by CLogMerger.

from messages import CMessage, CMessageTextQueue, CLogMessageSource
import threading
import legacy

_LOGGER = None

def get():
    global _LOGGER
    if _LOGGER == None: _LOGGER = CInternalLogger()
    return _LOGGER

class CInternalLogger(CLogMessageSource):
    def __init__(self):
        # Modelled like CProcess: messages, errors
        self.messages = legacy.deque(maxlen=500)
        self.errors = legacy.deque(maxlen=200)
        self.srcid = "castcontrol"
        self._lock = threading.Lock()

    def getLogMessages(self): # CLogMessageSource
        return self.getMessages(True, True)

    def isSameLogMessageSource(self, aObject): #CLogMessageSource
        return aObject == self

    # This is a CMessageTextQueue compatible method
    def getMessages(self, stdout=True, stderr=True):
        msgs = []
        try:
            self._lock.acquire(True)
            if stdout: msgs += self.messages
            if stderr: msgs += self.errors
            self.messages = []
            self.errors = []
        finally:
            self._lock.release()
        return msgs

    def log(self, msg):
        try:
            self._lock.acquire(True)
            self.messages.append(CMessage(self.srcid, msg))
        finally:
            self._lock.release()

    def warn(self, msg):
        try:
            self._lock.acquire(True)
            self.errors.append(CMessage(self.srcid, msg, msgtype=CMessage.WARNING))
        finally:
            self._lock.release()

    def error(self, msg):
        try:
            self._lock.acquire(True)
            self.errors.append(CMessage(self.srcid, msg, msgtype=CMessage.ERROR))
        finally:
            self._lock.release()

    def addMessage(self, cmsg):
        try:
            self._lock.acquire(True)
            if cmsg.msgtype == CMessage.ERROR or cmsg.msgtype == CMessage.WARNING:
                self.errors.append(cmsg)
            else: self.messages.append(cmsg)
        finally:
            self._lock.release()

class CStdoutLogger(CInternalLogger):
    def __init__(self):
        CInternalLogger.__init__(self)

    def log(self, msg):
        CInternalLogger.log(self, msg)
        print msg

    def warn(self, msg):
        CInternalLogger.warn(self, msg)
        print "!", msg

    def error(self, msg):
        CInternalLogger.error(self, msg)
        print "!!!!!", msg

    def addMessage(self, cmsg):
        CInternalLogger.addMessage(self, cmsg)
