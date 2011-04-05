#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jan 2010 
#
# Internal logging for CAST Control.
#
# The logger has a "message source" intreface that is used by CLogMerger.

from messages import CMessage
import legacy

_LOGGER = None

def get():
    global _LOGGER
    if _LOGGER == None: _LOGGER = CInternalLogger()
    return _LOGGER

class CInternalLogger(object):
    def __init__(self):
        # Modelled like CProcess: messages, errors
        self.messages = legacy.deque(maxlen=500)
        self.errors = legacy.deque(maxlen=200)
        self.srcid = "castcontrol"

    def getMessages(self, clear=True):
        msgs = list(self.messages)
        if clear: self.messages.clear()
        return msgs

    def getErrors(self, clear=True):
        msgs = list(self.errors)
        if clear: self.errors.clear()
        return msgs

    def log(self, msg):
        self.messages.append(CMessage(self.srcid, msg))

    def warn(self, msg):
        self.errors.append(CMessage(self.srcid, msg, msgtype=CMessage.WARNING))

    def error(self, msg):
        self.errors.append(CMessage(self.srcid, msg, msgtype=CMessage.ERROR))

    def addMessage(self, cmsg):
        if cmsg.msgtype == CMessage.ERROR or cmsg.msgtype == CMessage.WARNING:
            self.errors.append(cmsg)
        else: self.messages.append(cmsg)

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
