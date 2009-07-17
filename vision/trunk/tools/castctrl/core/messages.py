#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009 

import sys, time, re
import itertools, heapq
from collections import deque

pyver = sys.version_info[0] * 100 + sys.version_info[1]
reColorEscape = re.compile("\x1b\\[\\d+m")
reColorEscapeSplit = re.compile("(\x1b\\[\\d+m)")

class CMessage(object):
    MESSAGE=0
    WARNING=1
    ERROR=2
    CASTLOG=10
    def __init__(self, message, msgtype=0, order=0):
        self.order = order
        self.time = time.time()
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
    ansiColor = {
            1:  ("<b>", "</b>"),
            2:  ("<font color=grey>", "</font>"),
            3:  ("<i>", "</i>"),
            4:  ("<u>", "</u>"),
            31: ("<font color=darkred>", "</font>"),
            32: ("<font color=darkgreen>", "</font>"),
            33: ("<font color=orangered>", "</font>"),
            34: ("<font color=blue>", "</font>"),
            35: ("<font color=magenta>", "</font>"),
            36: ("<font color=darkcyan>", "</font>"),
            37: ("<font color=gray>", "</font>"),
            38: ("<font color=gray>", "</font>")
        }
    def __init__(self):
        pass

    def paint(self, msg):
        parts = reColorEscapeSplit.split(msg)
        intag = None
        for i, p in enumerate(parts):
            if not p.startswith("\x1b["):
                parts[i] = parts[i].replace("<", "&lt;")
            else:
                code = int(p[2:-1])
                if code >= 40 and code < 50: code -= 10
                elif code >= 90 and code < 100: code -= 60
                elif code >= 100 and code < 109: code -= 70
                elif code > 20 and code < 30: code -= 20
                if not CAnsiPainter.ansiColor.has_key(code): newtag = None
                else: newtag = CAnsiPainter.ansiColor[code]
                if newtag == None:
                    if intag != None: parts[i] = intag[1] # close tag
                    else: parts[i] = ""
                else:
                    if intag == None: parts[i] = newtag[0]
                    elif intag[0] != newtag[0]: parts[i] = intag[1] + newtag[0]
                    else: parts[i] = ""
                intag = newtag
        if intag != None: parts.append(intag[1])
        msg = "".join(parts) + " "
        return msg

class CMessageSource(object):
    def __init__(self, process):
        self.process = process
        self.tmLastSeen = 0
        self.tmLastPop = 0
        self.yieldMessages = True
        self.yieldErrors = True

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

class CLogMerger(object):
    def __init__(self):
        self.messages = [] # buffer with sorted messages
        self.sources = []

    def clearBuffer(self):
       self.messages = []

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
        if len(self.sources) < 1: return
        its = []
        for s in self.sources: its.extend(s.getIterators())
        if len(its) > 0:
            if pyver >= 206:
                for msg in heapq.merge(*its): self.messages.append(msg)
            else:
                def xmerge3(*ln):
                    heap = []
                    for i in itertools.chain(*ln): heap.append(i)
                    heapq.heapify(heap)
                    while len(heap): yield heapq.heappop(heap)

                for msg in xmerge3(*its): self.messages.append(msg)

class CInternalLogger(object):
    def __init__(self):
        # Modelled like CProcess
        self.messages = deque() # 2.6 deque(maxlen=500)
        self.errors = deque() # 2.6 deque(maxlen=200)

    def log(self, msg):
        self.messages.append(CMessage(msg))

    def warn(self, msg):
        self.errors.append(CMessage(msg, msgtype=CMessage.WARNING))

    def error(self, msg):
        self.errors.append(CMessage(msg, msgtype=CMessage.ERROR))

