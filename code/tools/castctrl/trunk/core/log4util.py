#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim

import os, sys, time
import stat
import options, logger

log4jlevels = ['ALL', 'TRACE', 'DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL', 'OFF']

class CLog4Config:
    def __init__(self):
        self._logDir = os.path.abspath("./logs")
        self._logFile = "cast-log.xml"
        self._serverConf = "cctmp.SimpleSocketServer.conf"
        self._clientConf = "cctmp.log4client.conf"
        self.serverXmlFileLevel = "TRACE"
        self.serverConsoleLevel = "LOG"
        self.serverPort = 48143
        self.serverHost = "localhost"
        # log4jproperties is a link that points to a client config file
        self.logPropLink = "log4j.properties"

    @property
    def logFile(self):
        return os.path.join(self._logDir, self._logFile)

    @property
    def serverConfigFile(self):
        return os.path.join(self._logDir, self._serverConf)

    @property
    def clientConfigFile(self):
        return os.path.join(self._logDir, self._clientConf)

    def setServerXmlFileLevel(self, levelName):
        self.serverXmlFileLevel = levelName

    def setServerConsoleLevel(self, levelName):
        self.serverConsoleLevel = levelName

    def setServerPort(self, port):
        self.serverPort = port

    def setXmlLogFilename(self, filename):
        path = os.path.abspath(filename)
        self._logDir = os.path.dirname(path)
        self._logFile = os.path.basename(path)
        if self._logFile == "":
            self._logFile = "cast-log.xml"

    def _prepareLogDir(self):
        try:
            if not os.path.exists(self._logDir):
                os.makedirs(self._logDir)
        except Exception as e:
            logger.get().error("%s" % e)

    def _prepareLogFile(self):
        opts = options.getCastOptions()
        self._prepareLogDir()
        head = opts.getSection("LOG4J.SimpleSocketServer.XMLLayout.head")
        user = "user"
        now = time.strftime("%H:%M:%S", time.localtime(time.time()))
        result = []
        for ln in head:
            ln = ln.replace('${USER}', user)
            ln = ln.replace('${NOW}', now)
            result.append(ln)

        try:
            f = open(self.logFile, 'w')
            f.write("\n".join(result))
            f.close()
        except Exception as e:
            logger.get().error("%s" % e)

    def prepareServerConfig(self):
        opts = options.getCastOptions()
        conf = opts.getSection("LOG4J.SimpleSocketServer.conf")
        result = []
        lnroot = ""
        for ln in conf:
            if ln.strip().startswith("log4j.rootLogger="):
                lnroot = ln.strip()
                continue
            result.append(ln.strip())

        if lnroot == "": lnroot = "log4j.rootLogger=TRACE"
        rootparts = lnroot.split(',')
        lnroot = [rootparts[0]]

        level = self.serverConsoleLevel
        if level != 'OFF':
            if len(rootparts) > 0: lnroot.append(rootparts[1])
            conf = opts.getSection("LOG4J.SimpleSocketServer.console")
            for ln in conf:
                ln = ln.replace('${LEVEL}', level)
                result.append(ln)

        level = self.serverXmlFileLevel
        if level != 'OFF':
            if len(rootparts) > 1: lnroot.append(rootparts[2])
            self._prepareLogFile()
            conf = opts.getSection("LOG4J.SimpleSocketServer.xmlfile")
            for ln in conf:
                ln = ln.replace('${LEVEL}', level)
                ln = ln.replace('${LOGFILE}', self.logFile)
                result.append(ln)

        result.insert(0, ",".join(lnroot))

        try:
            f = open(self.serverConfigFile, "w")
            f.write("\n".join(result))
            f.close()
        except Exception as e:
            logger.get().error("%s" % e)

    def prepareClientConfig(self, console = True, socketServer = True):
        opts = options.getCastOptions()
        clientfile = self.clientConfigFile
        conf = opts.getSection("LOG4J.client.conf")
        result = []
        lnroot = ""
        for ln in conf:
            if ln.strip().startswith("log4j.rootLogger="):
                lnroot = ln.strip()
                continue
            result.append(ln.strip())

        if lnroot == "": lnroot = "log4j.rootLogger=TRACE"
        rootparts = lnroot.split(',')
        lnroot = [rootparts[0]]

        level = self.serverConsoleLevel
        if console and level != 'OFF':
            if len(rootparts) > 0: lnroot.append(rootparts[1])
            conf = opts.getSection("LOG4J.client.console")
            for ln in conf:
                ln = ln.replace('${LEVEL}', level)
                result.append(ln)

        if socketServer:
            if len(rootparts) > 1: lnroot.append(rootparts[2])
            conf = opts.getSection("LOG4J.client.socket")
            for ln in conf:
                #ln = ln.replace('${LEVEL}', level) # NOT YET; MAX(serverConsoleLevel, serverXmlFileLevel)
                ln = ln.replace('${PORT}', self.serverPort)
                ln = ln.replace('${HOST}', self.serverHost)
                result.append(ln)

        result.insert(0, ",".join(lnroot))

        self._prepareLogDir()
        try:
            f = open(clientfile, "w")
            f.write("\n".join(result))
            f.close()
        except Exception as e:
            logger.get().error("%s" % e)

        try:
            if os.path.exists(self.logPropLink):
                st = os.lstat(self.logPropLink)
                if not stat.S_ISLNK(st.st_mode):
                    os.rename(self.logPropLink, os.tempnam(self._logDir, self.logPropLink))
                os.remove(self.logPropLink)
            os.symlink(clientfile, self.logPropLink)
        except Exception as e:
            logger.get().error("%s" % e)

