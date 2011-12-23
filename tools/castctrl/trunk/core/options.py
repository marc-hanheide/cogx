#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009 

import os, os.path
import re
import optdefault
import glob

regSimple = re.compile (r"\$([a-z_0-9]+)", re.IGNORECASE)
regSimpleBrace = re.compile (r"\${([a-z_0-9]+)}", re.IGNORECASE)
regVarSet = re.compile (r"^\s*([a-z_0-9]+)\s*=(.*)$", re.IGNORECASE)

startup_environ = os.environ.copy()

# http://standards.freedesktop.org/basedir-spec/basedir-spec-latest.html
if startup_environ.has_key("XDG_CONFIG_HOME"): CONFIG_DIR=startup_environ["XDG_CONFIG_HOME"]
elif startup_environ.has_key("HOME"): CONFIG_DIR=startup_environ["HOME"] + "/.config"
else: CONFIG_DIR="~/.config"
CONFIG_DIR = os.path.abspath(os.path.join(CONFIG_DIR, "CASTControl"))

# Factory methods for global singletons with settings
_castOptions = None
def getCastOptions():
    global _castOptions
    if _castOptions == None:
        _castOptions = CCastOptions()
    return _castOptions

_userOptions = None
def getUserOptions():
    global _userOptions
    if _userOptions == None:
        _userOptions = CUserOptions()
    return _userOptions

# Env-var expander
def _xe(shexpr, env=None, keepUnknown=False):
    if env == None: env = startup_environ
    for rx in [regSimple, regSimpleBrace]:
        mos = [mo for mo in rx.finditer(shexpr)]
        mos.reverse()
        for m in mos:
            if env.has_key(m.group(1)): v = env[m.group(1)]
            elif keepUnknown: v = "${%s}" % m.group(1)
            else: v = ""
            shexpr = shexpr.replace(m.group(0), v)

    return shexpr


def _getEnvVar(variable, env):
    if env.has_key(variable):
        return env[variable]
    return None


class CUserOptions(object):
    def __init__(self):
        self.textEditCmd = "gvim --servername GVIM --remote-silent %l[+:] %s"
        self.terminalCmd = "gnome-terminal --working-directory='%s'" 
        self.maxMainLogLines = 500
        self.maxBuildLogLines = 500
        self._config = ["[USEROPTIONS]"] + optdefault.useroptions.split("\n")
        self.modified = False

    @property
    def configFile(self):
        return os.path.join(CONFIG_DIR, "user.conf")

    def _getString(self, line):
        return line[line.find("=")+1:].strip()

    def _getInt(self, line, vmin=None, vmax=None):
        s = line[line.find("=")+1:].strip()
        try: v = int(s)
        except: v = 0
        if vmin != None and v < vmin: v = vmin
        if vmax != None and v > vmax: v = vmax
        return v

    def loadConfig(self):
        filename = self.configFile
        if not os.path.exists(filename):
            self.modified = True
            return
        f = open(filename, "r")
        self._config = [ ln.rstrip() for ln in f.readlines() ]
        f.close()
        section = None
        for ln in self._config:
            l = ln.split('#')[0]
            l = l.strip()
            if l == "[USEROPTIONS]": section = "USER"
            elif l.startswith('['): section = None
            else:
                if section == "USER":
                    if ln.startswith("EDITOR="): self.textEditCmd = self._getString(ln)
                    if ln.startswith("TERMINAL="): self.terminalCmd = self._getString(ln)
                    if ln.startswith("MAINLOGLINES="): self.maxMainLogLines = self._getInt(ln, 100, 10000)
                    if ln.startswith("BUILDLOGLINES="): self.maxBuildLogLines = self._getInt(ln, 100, 10000)

    def saveConfig(self):
        filename = self.configFile
        if not os.path.exists(CONFIG_DIR):
            os.makedirs(CONFIG_DIR, 0700)
        f = open(filename, "w")
        for ln in self._config:
            f.write(ln); f.write("\n")
        f.close()
        self.modified = False


class CCastOptions(object):
    def __init__(self):
        self.mruCfgPlayer = []
        self.mruCfgGolem = []
        self.mruCfgCast = []
        self.mruCfgHosts = []
        self.options = {}
        self.environmentScriptDefault = [s for s in optdefault.environment.split("\n")]
        self._environmentDefault = None
        self._environscript = None
        self._xenviron = None
        self._tempNewEnv = None # temp environ used for merging
        self.cleanupScript = [s.lstrip() for s in optdefault.cleanup.split("\n")]
        self.codeRootDir = os.path.abspath('.')

        self.confSection = {} # other configuration sections
        self.parseConfigLines(optdefault.log4joptions.split("\n"))

        self.configFileInfo = None

    @property
    def environscript(self):
        if self._environscript == None:
            self._environscript = [ln for ln in self.environmentScriptDefault]
        return self._environscript

    @property
    def environ(self):
        if self._xenviron == None:
            self._xenviron = self._mergeEnvironment(startup_environ, self.environscript)
        return self._xenviron

    @property
    def environmentDefault(self):
        if self._environmentDefault == None:
            self._environmentDefault = self._mergeEnvironment(startup_environ, self.environmentScriptDefault)
        return self._environmentDefault

    @property
    def envVarsFromScript(self):
        evars = self._mergeEnvironment({}, self.environscript)
        return evars.keys()

    def xe(self, shexpr, environ=None):
        if not environ:
            environ = self.environ
        return _xe(shexpr, environ)

    def getEnvVar(self, variable, environ=None):
        if not environ:
            environ = self.environ
        return _getEnvVar(variable, environ)

    def getDefaultEnvVar(self, variable):
        return _getEnvVar(variable, self.environmentDefault)

    def getExtendedEnviron(self, defaults=None):
        """
        Expand the 'shell' expression shexpr in the environment self.environ.
        If there are variables in defaults that are not in self.environ,
        self.environ will be recreated: first the missing variables will be
        defined and the the self.environscript will be re-evaluated.
        """
        if not defaults:
            return self.environ.copy()

        defaults = [ l.strip() for l in defaults.split("\n") if l.strip() != "" ]
        if len(defaults) < 1:
            return self.environ.copy()

        # Parse defaults to discover missing variables
        defaults = self._parseEnvironmentScript(defaults)
        enew = {}
        enew_order = []
        for (k, v, isPath) in defaults:
            if not k in self.environ:
                v = _xe(v, self.environ, keepUnknown=True)
                if isPath: v = self._cleanPathList(v)
                enew[k] = v
                enew_order.append(k)
        if len(enew) < 1:
            return self.environ.copy()

        # Rebuild the environment: strtup, defaults, environscript
        edef = self._mergeEnvironment(startup_environ, [])
        for k in enew_order:
            if k in enew:
                edef[k] = _xe(enew[k], edef)
        edef = self._mergeEnvironment(edef, self.environscript)

        return edef

    def parseConfigLines(self, lines):
        section = None
        for ln in lines:
            l = ln.split('#')[0]
            l = l.strip()
            if l == "[ENVIRONMENT]":
                self._environscript = []
                section = self._environscript
            elif l == "[CLEANUP-SCRIPT]":
                self.cleanupScript = []
                section = self.cleanupScript
            elif l.startswith('[') and l.endswith(']'):
                if l == "[USEROPTIONS]": section = None # XXX old version kept user options in same file
                else:
                    section = []
                    self.confSection[l.strip(" []")] = section
            elif section != None:
                section.append(ln.rstrip())

    def loadConfig(self, filename):
        if not os.path.exists(filename): return
        self._xenviron = None
        f = open(filename, "r")
        self.parseConfigLines(f.readlines())
        f.close()
        filename = os.path.abspath(filename)
        statinfo = os.stat(filename)
        self.configFileInfo = [filename, statinfo]

    # Check if the config file has changed on disk and reload
    def checkConfigFile(self):
        if self.configFileInfo == None: return
        filename = self.configFileInfo[0]
        oldstat = self.configFileInfo[1]
        statinfo = os.stat(filename)
        if oldstat.st_mtime != statinfo.st_mtime:
            self.loadConfig(filename)
            self.configEnvironment()


    def _get_mru_lists(self):
        return [ ("MRU-CAST", self.mruCfgCast), ("MRU-HOSTS", self.mruCfgHosts) ]


    def loadHistory(self, configParser):
        p = configParser
        hists = self._get_mru_lists()

        for mru in hists:
            section = mru[0]
            if not p.has_section(section): continue
            items = sorted(p.items(section))
            mlist = mru[1]
            try: # clear list
                while 1: mlist.pop()
            except: pass
            mlist += [ v[1] for v in items ]

        section = "OPTIONS"
        if p.has_section(section):
            items = p.items(section)
            for v in items:
                self.options[v[0]] = v[1]


    def getSection(self, sectionName):
        if self.confSection.has_key(sectionName): return self.confSection[sectionName]
        return []

    # this should be called only when the config file doesnt exist
    def saveConfig(self, afile):
        f = afile
        f.write("[ENVIRONMENT]\n")
        for ln in self.environscript:
            f.write(ln); f.write("\n")
        f.write("[CLEANUP-SCRIPT]\n")
        for ln in self.cleanupScript:
            f.write(ln); f.write("\n")

        # custom sections
        for k,v in self.confSection.iteritems():
            f.write("[%s]\n" % k)
            for ln in v:
                f.write(ln); f.write("\n")


    def saveHistory(self, configParser):
        p = configParser
        hists = self._get_mru_lists()

        for mru in hists:
           section = mru[0]
           p.remove_section(section)
           p.add_section(section)
           for i,h in enumerate(mru[1]):
              p.set(section, "%03d" % (i+1), "%s" % h)

        section = "OPTIONS"
        if not p.has_section(section):
            p.add_section(section)
        for k,v in self.options.iteritems():
            p.set(section, "%s" % k, "%s" % v)


    def _globFileList(self, globTag):
        globTag = globTag.replace("<glob>", "").replace("</glob>", "")
        globExpr = _xe(globTag, self._tempNewEnv)
        return glob.glob(globExpr)

    def _readPathList(self, lineIterator, separator=":"):
        paths = []
        for stm in lineIterator:
            stm = stm.split('#')[0]
            stm = stm.strip()
            if len(stm) < 1: continue
            if stm == "</pathlist>": break
            if stm.startswith("<glob>"):
                paths += self._globFileList(stm)
            else: paths.append(stm)
        paths = separator.join(paths)
        return paths

    def _cleanPathList(self, pathString, delim=":"):
        p = pathString.replace(": :", ":")
        p = p.replace("::", ":")
        p = p.strip(": ")
        return p

    def _readMultiLine(self, lineIterator):
        res = []
        for stm in lineIterator:
            stm = stm.split('#')[0]
            stm = stm.strip()
            if len(stm) < 1: continue
            if stm == "</multiline>": break
            res.append(stm)
        return " ".join(res)


    # Returns a list [ (key, value, isPath), ... ]
    def _parseEnvironmentScript(self, lineList):
        iexpr = lineList.__iter__()
        elist = []
        for stm in iexpr:
            stm = stm.split('#')[0]
            stm = stm.strip()
            if len(stm) < 1: continue
            mo = regVarSet.match(stm)
            isPath = False
            if mo != None:
                lhs, rhs = mo.group(1), mo.group(2)
                if rhs == "<pathlist>":
                    rhs = self._readPathList(iexpr, ":")
                    isPath = True
                elif rhs == "<multiline>":
                    rhs = self._readMultiLine(iexpr)
                rhs = rhs.replace("[PWD]", self.codeRootDir)

                elist.append( (lhs, rhs, isPath) )
            else: print "Invalid ENV expression:", stm

        return elist

    def _mergeEnvironment(self, env, expressionList):
        self._tempNewEnv = env.copy()

        elist = self._parseEnvironmentScript(expressionList)
        for (lhs, rhs, isPath) in elist:
            rhs = _xe(rhs, self._tempNewEnv)
            if isPath: rhs = self._cleanPathList(rhs)
            self._tempNewEnv[lhs] = rhs

        return self._tempNewEnv


    def configEnvironment(self):
        newenv = self.environ
        for k,v in newenv.iteritems():
            os.environ[k] = v
        return

    def _storeMru(self, list, item):
        if item in list: list.remove(item)
        list.insert(0, item)

    def addPlayerConfig(self, filename):
        self._storeMru(self.mruCfgPlayer, filename)
        pass

    def addGolemConfig(self, filename):
        self._storeMru(self.mruCfgGolem, filename)
        pass

    def addCastConfig(self, filename):
        self._storeMru(self.mruCfgCast, filename)
        pass

    def addHostsConfig(self, filename):
        self._storeMru(self.mruCfgHosts, filename)
        pass

    def getOption(self, key):
        key = key.lower() # ConfigParser uses lower case in the default optionxform()
        if self.options.has_key(key): return self.options[key]
        return ""

    def setOption(self, key, value):
        key = key.lower() # ConfigParser uses lower case in the default optionxform()
        self.options["%s" % key] =  "%s" % value

