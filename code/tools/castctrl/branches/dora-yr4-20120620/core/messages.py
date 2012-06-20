#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009 

import sys, time, re
import itertools, heapq
import legacy
import threading

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


# 2012-03-29
# A process writes raw text to a CMessageTextQueue.
# A message source reads messages from CMessageTextQueue, converted to CMessage
class CMessageTextQueue(object):
    STDOUT = 1
    STDERR = 2
    FLUSH  = 3 # flushing after the process is dead
    def __init__(self, sourceId):
        self.textBlocks = []          # (timestamp, text)
        self.sourceId = sourceId      # id of the process
        self.order = 0                # order of the messages for sorting when timestamp is the same
        self.messageProcessor = None  # Some servers (log4j) could do additional message processing
        self.lastLineIsBlank = False  # for compression of blank lines
        self.lineCount = 0
        self.lineCountLimit = 1000
        self._lock = threading.Lock()

    # streamType: STDOUT, STDERR, FLUSH
    def addText(self, streamType, lines):
        tmst = time.time()
        text = []
        for line in lines:
            blank = len(line) < 1 or line.isspace()
            if self.lastLineIsBlank and blank: continue
            self.lastLineIsBlank = blank
            text.append(line)
            if len(text) > self.lineCountLimit:
                print "addText: too many lines added (%d/%d)" % (len(lines), self.lineCountLimit)
                break

        if len(text) < 1:
            return 0

        try:
            self._lock.acquire(True)
            self.textBlocks.append( (time.time(), text, streamType) )
            self.lineCount += len(text)
            dropped = 0
            while self.lineCount > self.lineCountLimit and len(self.textBlocks) > 1:
                dropped += len(self.textBlocks[0][1])
                self.lineCount -= len(self.textBlocks[0][1])
                self.textBlocks = self.textBlocks[1:]
            if dropped:
                print "addText: buffer full. %d lines dropped." % dropped
        finally:
            self._lock.release()

        if len(text) > 100: print self.sourceId, len(text), "in", "%.6f" % (time.time() - tmst)
        return len(text)

    # could this be a generator?
    def getMessages(self, stdout=True, stderr=True):
        tmst = time.time()
        msgs = []
        try:
            self._lock.acquire(True)
            blocks = self.textBlocks
            self.textBlocks = []
            self.lineCount = 0
        finally:
            self._lock.release()

        for (tm, block, streamType) in blocks:
            if streamType == CMessageTextQueue.STDOUT:
                if not stdout: continue
                mtype = CMessage.MESSAGE
            else:
                if not stderr: continue
                if streamType == CMessageTextQueue.FLUSH:
                    mtype = CMessage.FLUSHMSG
                else: mtype = CMessage.ERROR

            for line in block:
                self.order += 1
                if self.messageProcessor:
                    (line, mtype) = self.messageProcessor.process(line, mtype)
                line = line.decode("utf-8", "replace")
                msgs.append(CMessage(self.sourceId, line, mtype, self.order, tm))

        if len(msgs) > 100: print self.sourceId, len(msgs), "out", "%.6f" % (time.time() - tmst)
        return msgs


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

class CLogMessageSource(object):
    def __init__(self):
        self._messageSinks = []

    # To be overriddend by the actual implementation
    def _getLogMessages(self):
        return []

    def pushLogMessages(self):
        msgs = self._getLogMessages()
        if len(msgs):
            for s in self._messageSinks:
                s.addMessages(msgs)

    def addSink(self, sink):
        self._messageSinks.append(sink)

    def removeSink(self, sink):
        keep = []
        for s in self._messageSinks:
            if s == sink: continue
            keep.append(s)
        self._messageSinks = keep

    # Detect if the object is associated with this CLogMessageSource.
    # Useful when the CLogMessageSource is a wrapper around the message producer.
    def isSameLogMessageSource(self, aObject):
        return aObject == self

    def setMessageProcessor(self, messageProcessor):
        pass

class CLogMessageSink(object):
    def __init__(self):
        self.maxlen = 10000
        self.messages = []
        self.fnMatches = None

    def clear(self):
        self.messages = []

    def addMessages(self, messages):
        self.messages += messages

    def getNewMessages(self, maxItems=0):
        if maxItems <= 0:
            msgs = self.messages
            self.messages = []
        else:
            msgs = self.messages[:maxItems]
            self.messages = self.messages[maxItems:]
        return msgs

    def hasMessages(self):
        return len(self.messages) > 0

class CLogMerger(object):
    def __init__(self):
        self.maxlen = 10000
        self.messages = [] # buffer with sorted messages
        self._sources = []
        self._sinks = []
        self.fnMatches = None
        self.filtered = self.messages
        self.current = 0
        self._lock = threading.Lock()

    def clearBuffer(self):
       self.messages = []
       self.filtered = self.messages
       self.current = 0

    def setFilter(self, fnFilter, lastMessages = -1): # fnFilter(message)
        try:
            self._lock.acquire(True)
            self.fnMatches = fnFilter
            if fnFilter == None: self.filtered = self.messages
            else:
                self.filtered = []
                self._applyFilter(self.messages)
            self.restart(lastMessages)

            for s in self._sinks: s.clear()
            self._pushNewMessages()
        finally:
            self._lock.release()

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

    def _pushNewMessages(self, maxItems=0):
        if maxItems < 1:
           msgs = self.filtered[self.current:]
           self.current = len(self.filtered)
        else:
           nc = self.current+maxItems
           msgs = self.filtered[self.current:nc]
           self.current = nc
           if self.current > len(self.filtered):
               self.current = len(self.filtered)

        for s in self._sinks:
            s.addMessages(msgs)

    def addSource(self, logSource):
        self.removeSource(logSource)
        sink = CLogMessageSink()
        logSource.addSink(sink)
        self._sources.append((logSource, sink))

    def removeSource(self, aObject):
        keep = []
        for s in self._sources:
            if s[0] == aObject or s[0].isSameLogMessageSource(aObject):
                s[0].removeSink(s[1])
                continue
            keep.append(s)
        self._sources = keep

    def findSource(self, aObject):
        for src,sink in self._sources:
            if src == aObject or src.isSameLogMessageSource(aObject): return src
        return None

    def hasSource(self, aObject):
        return self.findSource(aObject) != None

    def removeAllSources(self):
        for s in self._sources:
            s[0].removeSink(s[1])
        self._sources = []

    def addSink(self, sink):
        self._sinks.append(sink)

    def merge(self):
        tmst = time.time()
        msgs = []
        try:
            self._lock.acquire(True)
            for src,sink in self._sources:
                src.pushLogMessages()
                msgs.extend(sink.getNewMessages())
            if len(msgs) < 1: return
            if len(msgs) > 500: print "got %d new messages in %.6f" % (len(msgs), time.time() - tmst)
            msgs.sort()
            if len(self.messages) < 1:
                self._setMessages(msgs)
                self._applyFilter(msgs)
                self._pushNewMessages()
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
            self._pushNewMessages()
        finally:
            self._lock.release()

