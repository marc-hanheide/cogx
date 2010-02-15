#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009 

import sys, time, re
import itertools, heapq
import legacy
from collections import deque

reColorEscape = re.compile("\x1b\\[[0-9;]+m")
reColorEscapeSplit = re.compile("(\x1b\\[[0-9;]+m)")

class CMessage(object):
    MESSAGE=0
    WARNING=1
    ERROR=2
    FLUSHMSG=3
    CASTLOG=10
    def __init__(self, message, msgtype=0, order=0, timestamp=time.time()):
        self.order = order
        self.time = timestamp
        self.message = message.rstrip()
        self.msgtype = msgtype

    def __cmp__(self, x):
        if self.time < x.time: return -1
        if self.time > x.time: return 1
        return 0

    def getText(self, timefmt = "%H:%M:%S"):
        if timefmt != None:
            stm = time.strftime("%H:%M:%S", time.localtime(self.time))
            return "%s %s" % (stm, self.message)
        return self.message

class CAnsiPainter(object):
    colorName = ["black", "darkred", "darkgreen", "orangered",
                 "blue", "magenta", "darkcyan", "gray", "gray", "black"]
    backgrName = ["lightgray", "lightred", "lightgreen", "yellow",
                  "lightblue", "lightpink", "lightcyan", "white", "white", "white"]

    def __init__(self):
        pass

    def paint(self, msg):
        parts = reColorEscapeSplit.split(msg)
        intag = None
        styles = {}
        laststyle = None
        for i, p in enumerate(parts):
            if not p.startswith("\x1b["):
                parts[i] = parts[i].replace("&", "&amp;").replace("<", "&lt;")
                continue

            codelist = p[2:-1].split(";")
            for code in codelist:
                code = int(codelist[0])
                if code >= 90 and code < 109: code -= 60
                if code < 30:
                    if code == 0: styles = {}
                    elif code == 1: styles["font-weight"] = "bold"
                    elif code == 2: styles.pop("font-weight", "")
                    elif code == 3: styles["font-style"] = "italic"
                    elif code == 4: styles["text-decoration"] = "underline"
                    elif code == 21: styles["text-decoration"] = "underline"
                    elif code == 22: styles.pop("font-weight", "")
                    elif code == 23: styles.pop("font-style", "")
                    elif code == 24: styles.pop("text-decoration", "")
                elif code >= 30 and code <= 39: styles["color"] = CAnsiPainter.colorName[code-30]
                elif code >= 40 and code <= 49: styles["background"] = CAnsiPainter.backgrName[code-40]

            newstyle = ";".join(["%s:%s" % (k, v) for k,v in styles.iteritems() if len(v) > 0])
            if newstyle == "": newstyle=None
            if newstyle == laststyle: parts[i] = ""
            else:
                if newstyle == None: stag = ""
                else: stag = "<span style='%s'>" % newstyle
                if laststyle == None: parts[i] = stag
                else: parts[i] = "</span>" + stag
                laststyle = newstyle
        if laststyle != None: tail = "</span>"
        else: tail = ""
        msg = "".join(parts) + tail
        return msg

class CMessageSource(object):
    def __init__(self, process):
        self.process = process
        self.tmLastSeen = 0
        self.tmLastPop = 0
        self.yieldMessages = True
        self.yieldErrors = True

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
            its.append(itertools.ifilter(lambda x: checkTime(x), self.process.messages))
        if self.yieldErrors:
            its.append(itertools.ifilter(lambda x: checkTime(x), self.process.errors))
        return its

    def restart(self):
        self.tmLastSeen = 0
        self.tmLastPop = 0

class CLogMerger(object):
    def __init__(self):
        self.messages = [] # buffer with sorted messages
        self.sources = []

    def clearBuffer(self):
       self.messages = []

    def getNewMessages(self, maxItems=0):
        """
        Return up to maxItems new messages and remove them from the queue.
        If maxItems < 1, all messages are returned.
        """
        if maxItems < 1:
            msgs = self.messages
            self.messages = []
        else:
            msgs = self.messages[:maxItems]
            self.messages = self.messages[maxItems:]
        return msgs

    def addSource(self, process):
        self.removeSource(process)
        src = CMessageSource(process)
        self.sources.append(src)
        return src

    def removeSource(self, process):
        srcs = [s for s in self.sources if s.process == process]
        for s in srcs: self.sources.remove(s)

    def hasSource(self, process):
        for s in self.sources:
            if s.process == process: return True
        return False

    def removeAllSources(self):
        self.sources = []

    def merge(self):
        """
        Merge messages from different sources into one queue.
        The messages in the destination queue are (mostly) sorted by timestamp.
        """
        if len(self.sources) < 1: return
        its = []
        # FIXME: queues that iterators iterate over may change while fnMerge is active
        # (source queues are filled in separate threads; messages remain in queues;)
        for s in self.sources: its.extend(s.getIterators())
        if len(its) > 0:
            fnMerge = legacy.getMergeFn()
            for msg in fnMerge(*its): self.messages.append(msg)

class CInternalLogger(object):
    def __init__(self):
        # Modelled like CProcess: messages, errors
        self.messages = legacy.deque(maxlen=500)
        self.errors = legacy.deque(maxlen=200)

    def log(self, msg):
        self.messages.append(CMessage(msg))

    def warn(self, msg):
        self.errors.append(CMessage(msg, msgtype=CMessage.WARNING))

    def error(self, msg):
        self.errors.append(CMessage(msg, msgtype=CMessage.ERROR))

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
