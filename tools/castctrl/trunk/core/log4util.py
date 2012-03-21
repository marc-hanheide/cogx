#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim

import os, sys, time
import stat
import options, logger
import re, messages

log4jlevels = ['ALL', 'TRACE', 'DEBUG', 'INFO', 'WARN', 'ERROR', 'FATAL', 'OFF']

class CLog4Config:
    def __init__(self):
        self._logDir = os.path.abspath("./logs")
        self._logFile = "cast-log.xml"
        self._serverConf = "cctmp.log4server.conf"
        self._clientConf = "cctmp.log4client.conf"
        self.serverXmlFileLevel = "TRACE"
        self.serverConsoleLevel = "LOG"
        self.serverPort = 48143
        self.serverHost = "localhost"
        # log4jproperties is a link that points to a client config file
        self.logPropLink = "log4j.properties"
        self.loggerLevelsFilename = ""

        serverAppenders = {
            'console': ('LOG4J.SimpleSocketServer.console', ), # params: level
            'xmlfile': ('LOG4J.SimpleSocketServer.xmlfile', ), # params: level, filename
        }

        # These settings names reflect the settings in default_log4jsrv.txt
        # ( appender-name, ini-section )
        servers = {}
        servers['console'] = {
            'id': 'console',
            'order': 0,
            'name': 'Console',
            'envvar': None, # no need to start it
            'appenders': None,
            'client-connect': ('LOG4J.client.console', ) # params: level
        }
        servers['socket-server'] = {
            'id': 'socket-server',
            'order': 10,
            'name': 'Log4j Socket Server',
            'envvar': 'CMD_LOG4J_SOCKET_SERVER',
            'appenders': serverAppenders,
            'client-connect': ('LOG4J.client.socket', ) # params: hostname
        }
        servers['cast-server'] = {
            'id': 'cast-server',
            'order': 11,
            'name': 'CAST Log Server',
            'envvar': 'CMD_LOG4J_CAST_SERVER',
            'appenders': serverAppenders,
            'client-connect': ('LOG4J.client.IceAppender', ) # params: hostname
        }
        self.servers = servers

        self.selectedServer = 'socket-server' # TODO: change to cast-server whwn 2.1.16 published
        self.startServer = True


    @property
    def logFile(self):
        # MUST be relative if used on a remote agent, so _logDir can't be used
        return self._logFile

    @property
    def logServerDir(self):
        return self._logDir

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
        #self._logDir = os.path.dirname(path)
        self._logFile = os.path.basename(path)
        if self._logFile == "":
            self._logFile = "cast-log.xml"

    def _prepareLogDir(self):
        try:
            if not os.path.exists(self._logDir):
                os.makedirs(self._logDir)
        except Exception as e:
            logger.get().error("%s" % e)

    def _removeComments(self, items, removeAll=False):
        clean = []
        for ln in items:
            ln = ln.strip()
            if ln.startswith("#"):
                if removeAll or not ln.startswith("#+"): continue
            clean.append(ln)
        return clean

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
        result = self._removeComments(result)

        try:
            f = open(os.path.join(self._logDir, self._logFile), 'w')
            f.write("\n".join(result))
            f.close()
        except Exception as e:
            logger.get().error("%s" % e)


    def _insertRootLogger(self, configLines):
        definedAppenders = {}
        for ln in configLines:
            if not ln.startswith("log4j.appender."): continue
            parts = ln.split('=')[0].split('.')
            definedAppenders[parts[2]] = 1
        definedAppenders = definedAppenders.keys()

        configLines.insert(0, ",".join(["log4j.rootLogger=TRACE"] + definedAppenders))
        pass


    def prepareServerConfig(self):
        result = []
        opts = options.getCastOptions()

        # define appenders
        sm = self.servers[self.selectedServer]
        appenders = sm['appenders']
        for k,appndr in appenders.items():
            result.append("#+ Section: " + appndr[0])
            level = self.serverConsoleLevel
            if k == 'console' and level != 'OFF':
                conf = opts.getSection(appndr[0])
                for ln in conf:
                    ln = ln.replace('${LEVEL}', level)
                    result.append(ln)

            level = self.serverXmlFileLevel
            if k == 'xmlfile' and level != 'OFF':
                self._prepareLogFile()
                conf = opts.getSection(appndr[0])
                for ln in conf:
                    ln = ln.replace('${LEVEL}', level)
                    ln = ln.replace('${LOGFILE}', self.logFile)
                    result.append(ln)

        # cleanup
        result = [ln.strip() for ln in result if not ln.strip().startswith("log4j.rootLogger=")]
        result = self._removeComments(result)

        # setup rootLogger
        self._insertRootLogger(result)

        self._prepareLogDir()
        try:
            f = open(self.serverConfigFile, "w")
            f.write("\n".join(result))
            f.close()
        except Exception as e:
            logger.get().error("%s" % e)

        serverLink = os.path.join(self.logServerDir, self.logPropLink)
        try:
            if os.path.exists(serverLink):
                st = os.lstat(serverLink)
                if not stat.S_ISLNK(st.st_mode):
                    os.rename(self.logPropLink, os.tempnam(self._logDir, "srv." + self.logPropLink))
                os.remove(serverLink)
            os.symlink(self.serverConfigFile, serverLink)
        except Exception as e:
            logger.get().error("%s" % e)


    def prepareClientConfig(self):
        opts = options.getCastOptions()
        sm = self.servers[self.selectedServer]
        result = []

        appndr = sm['client-connect']
        conf = opts.getSection(appndr[0])
        result.append("#+ Section: " + appndr[0])
        for ln in conf:
            if sm['id'] == 'console':
                ln = ln.replace('${LEVEL}', self.serverConsoleLevel)
            else:
                ln = ln.replace('${PORT}', self.serverPort)
                ln = ln.replace('${HOST}', self.serverHost)
            result.append(ln)

        if not os.path.exists(self.loggerLevelsFilename):
            if self.loggerLevelsFilename != None and self.loggerLevelsFilename != "":
                logger.get().error(
                        "File '%s' not found. Using default logging levels."
                        % self.loggerLevelsFilename)  
        else:
            f = open(self.loggerLevelsFilename)
            lines = self._removeComments(f.readlines())
            f.close()
            for ln in lines:
                ln = ln.strip()
                if ln == "": continue
                if not ln.startswith("#"):
                    result.append("log4j.logger." + ln)

        # cleanup
        result = [ln.strip() for ln in result if not ln.strip().startswith("log4j.rootLogger=")]
        result = self._removeComments(result)

        # setup rootLogger
        self._insertRootLogger(result)

        clientfile = self.clientConfigFile
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
                    os.rename(self.logPropLink, os.tempnam(self._logDir, "cli." + self.logPropLink))
                os.remove(self.logPropLink)
            os.symlink(clientfile, self.logPropLink)
        except Exception as e:
            logger.get().error("%s" % e)


class CLog4MessageProcessor:
    reMsg = re.compile("^\s*\[[A-Z]+\s+([^:]+):")
    def __init__(self):
        self.colors = {}
        self.nextColor = 0

    def paint(self, message, id):
        co = self.colors[id] % 10
        return "\x1b[%dm%s\x1b[0m" % (co + 30, message)

    def process(self, message, mtype):
        if mtype == messages.CMessage.MESSAGE:
            mo = CLog4MessageProcessor.reMsg.match(message)
            if mo != None: # re.match
                mtype = messages.CMessage.CASTLOG
                id = mo.group(1)
                if not id in self.colors:
                    self.colors[id] = self.nextColor
                    self.nextColor += 1
                message = self.paint(message, id)

        return (message, mtype)
