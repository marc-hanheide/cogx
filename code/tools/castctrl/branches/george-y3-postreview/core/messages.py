#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009 

import sys, time, re
import itertools, heapq
import legacy

reColorEscape = re.compile("\x1b\\[[0-9;]+m")
reColorEscapeSplit = re.compile("(\x1b\\[[0-9;]+m)")
reExtractComponent = re.compile("\\[([a-zA-Z]*\\s+)?([^:\\[\\] ]+):") # [DEBUG component: ...]

class CMessage(object):
    MESSAGE=0
    WARNING=1
    ERROR=2
    FLUSHMSG=3
    CASTLOG=10
    def __init__(self, source, message, msgtype=0, order=0, timestamp=None):
        if timestamp == None: timestamp = time.time()
        self.source = source
        self.order = order
        self.time = timestamp
        self.message = message.rstrip()
        self.msgtype = msgtype
        self.component = ""
        if self.msgtype == CMessage.CASTLOG:
            mo = reExtractComponent.search(message)
            if mo != None:
                self.component = mo.group(2)

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
            if newstyle == laststyle: parts[i] = ""
            else:
                if newstyle == "": stag = ""
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
        self.yieldMessages = True
        self.yieldErrors = True

    def getMessages(self, clear=True):
        msgs = []
        if self.yieldMessages: msgs += self.process.getMessages(clear)
        if self.yieldErrors: msgs += self.process.getErrors(clear)
        return msgs

class CLogMerger(object):
    def __init__(self):
        self.maxlen = 10000
        self.messages = [] # buffer with sorted messages
        self._sources = []
        self.fnMatches = None
        self.filtered = self.messages
        self.current = 0

    def clearBuffer(self):
       self.messages = []
       self.filtered = self.messages
       self.current = 0

    def setFilter(self, fnFilter, lastMessages = -1): # fnFilter(message)
        self.fnMatches = fnFilter
        if fnFilter == None: self.filtered = self.messages
        else:
            self.filtered = []
            self._applyFilter(self.messages)
        self.restart(lastMessages)

    def _setMessages(self, newList):
        if self.messages == self.filtered: self.filtered = newList
        self.messages = newList

    def _applyFilter(self, newItems):
        if self.fnMatches == None: return
        if self.filtered == self.messages: self.filtered = []
        for it in newItems:
            if self.fnMatches(it):
                self.filtered.append(it)

    def _checkLength(self):
        if len(self.messages) <= self.maxlen: return
        nl = int(self.maxlen * 0.9)
        delta = len(self.messages) - nl
        self.messages = self.messages[-nl:]
        if self.filtered == self.messages:
            self.current -= delta
            return

        if len(self.filtered) <= self.maxlen: return
        nl = int(self.maxlen * 0.9)
        delta = len(self.filtered) - nl
        self.filtered = self.filtered[-nl:]
        self.current -= delta

    def restart(self, lastMessages=-1):
        if lastMessages > 0: self.current = max(0, len(self.filtered) - lastMessages)
        else: self.current = 0

    def getNewMessages(self, maxItems=0):
        if maxItems < 1:
           msgs = self.filtered[self.current:]
           self.current = len(self.filtered)
        else:
           nc = self.current+maxItems
           msgs = self.filtered[self.current:nc]
           self.current = nc
           if self.current > len(self.filtered):
               self.current = len(self.filtered)
        return msgs

    def addSource(self, process):
        # print type(process), type(process).__bases__
        if type(process) == CMessageSource or CMessageSource in type(process).__bases__:
            # print "A CMessageSource", type(process)
            src = process
            self.removeSource(src.process)
        else:
            self.removeSource(process)
            src = CMessageSource(process)
        self._sources.append(src)
        return src

    def removeSource(self, process):
        if type(process) == CMessageSource or CMessageSource in type(process).__bases__:
            process = process.process
        srcs = [s for s in self._sources if s.process == process]
        for s in srcs: self._sources.remove(s)

    def hasSource(self, process):
        for s in self._sources:
            if s.process == process: return True
        return False

    def removeAllSources(self):
        self._sources = []

    def merge(self):
        msgs = []
        for s in self._sources: msgs.extend(s.getMessages(clear=True))
        if len(msgs) < 1: return
        # print len(msgs), "new messages"
        msgs.sort()
        if len(self.messages) < 1:
            self._setMessages(msgs)
            self._applyFilter(msgs)
            return
        first = msgs[0]
        i = len(self.messages) - 1
        while i > 0 and first < self.messages[i]: i -= 1
        resorted = self.messages[i+1:]
        # if len(resorted) > 0: print len(resorted), "resorted"
        merged = self.messages[:i+1] + sorted(resorted + msgs)
        self._setMessages(merged)
        self._applyFilter(msgs)
        self._checkLength()

