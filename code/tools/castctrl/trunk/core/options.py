#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009 

import os, os.path
import re
import optdefault

regSimple = re.compile (r"\$([a-z_0-9]+)", re.IGNORECASE)
regSimpleBrace = re.compile (r"\${([a-z_0-9]+)}", re.IGNORECASE)
regVarSet = re.compile (r"^\s*([a-z_0-9]+)\s*=(.*)$", re.IGNORECASE)

startup_environ = os.environ.copy()

def _xe(shexpr, env=None):
    if env == None: env = startup_environ
    for rx in [regSimple, regSimpleBrace]:
        mos = [mo for mo in rx.finditer(shexpr)]
        mos.reverse()
        for m in mos:
            if env.has_key(m.group(1)): v = env[m.group(1)]
            else: v = ""
            shexpr = shexpr.replace(m.group(0), v)

    return shexpr


class CUserOptions(object):
    def __init__(self):
        self.textEditCmd = "gvim --servername CAST --remote-silent %l[+:] %s"
        self.terminalCmd = "gnome-terminal --working-directory='%s'" 

    def loadConfig(self, filename):
        if not os.path.exists(filename): return
        f = open(filename, "r")
        section = None
        for ln in f.readlines():
            l = ln.split('#')[0]
            l = l.strip()
            if l == "[USEROPTIONS]": section = "USER"
            elif l.startswith('['): section = None
            else:
                if section == "USER":
                    if ln.startswith("EDITOR="): self.textEditCmd = ln[7:].strip()
                    if ln.startswith("TERMINAL="): self.terminalCmd = ln[9:].strip()
        f.close()

class CCastOptions(object):
    def __init__(self):
        self.mruCfgPlayer = []
        self.mruCfgCast = []
        self.mruCfgHosts = []
        self.options = {}
        self.environmentDefault = [s.lstrip() for s in optdefault.environment.split("\n")]
        self._environscript = None
        self._xenviron = None
        self.cleanupScript = [s.lstrip() for s in optdefault.cleanup.split("\n")]
        self.codeRootDir = os.path.abspath('.')

    @property
    def environscript(self):
        if self._environscript == None:
            self._environscript = [ln for ln in self.environmentDefault]
        return self._environscript

    @property
    def environ(self):
        if self._xenviron == None:
            self._xenviron = self._mergeEnvironment(startup_environ, self.environscript)
        return self._xenviron

    def xe(self, shexpr):
        return _xe(shexpr, self.environ)

    def loadConfig(self, filename):
        if not os.path.exists(filename): return
        self._xenviron = None
        f = open(filename, "r")
        section = None
        for ln in f.readlines():
            l = ln.split('#')[0]
            l = l.strip()
            if l == "[ENVIRONMENT]":
                self._environscript = []
                section = self._environscript
            elif l == "[CLEANUP-SCRIPT]":
                self.cleanupScript = []
                section = self.cleanupScript
            elif l.startswith('['): section = None
            elif section != None:
                section.append(ln.rstrip())
        f.close()

    def loadHistory(self, filename):
        if not os.path.exists(filename): return
        f = open(filename, "r")
        options = []
        section = None
        for ln in f.readlines():
            l = ln.split('#')[0]
            l = l.strip()
            if l == "[MRU-CAST]": section = self.mruCfgCast
            elif l == "[MRU-PLAYER]": section = self.mruCfgPlayer
            elif l == "[MRU-HOSTS]": section = self.mruCfgHosts
            elif l == "[OPTIONS]": section = options
            elif l.startswith('['): section = None
            elif section != None:
                section.append(ln.rstrip())

        f.close()

        for ln in options:
            expr = ln.split("=", 2)
            if len(expr) != 2: continue
            expr = [ e.strip() for e in expr ]
            if expr[0].startswith("#") or expr[0] == "": continue
            self.options[expr[0]] = expr[1]
            

    # this should be called only when the config file doesnt exist
    def saveConfig(self, afile):
        f = afile
        f.write("[ENVIRONMENT]\n")
        for ln in self.environscript:
            f.write(ln); f.write("\n")
        f.write("[CLEANUP-SCRIPT]\n")
        for ln in self.cleanupScript:
            f.write(ln); f.write("\n")
        # FIXME: temporary location for user options; move to a file in home dir
        f.write("[USEROPTIONS]\n")
        f.write(optdefault.useroptions)

    def saveHistory(self, afile):
        f = afile
        f.write("[MRU-CAST]\n")
        for ln in self.mruCfgCast:
            f.write(ln); f.write("\n")
        f.write("[MRU-PLAYER]\n")
        for ln in self.mruCfgPlayer:
            f.write(ln); f.write("\n")
        f.write("[MRU-HOSTS]\n")
        for ln in self.mruCfgHosts:
            f.write(ln); f.write("\n")
        f.write("[OPTIONS]\n")
        for k,v in self.options.iteritems():
            f.write("%s=%s\n" % (k,v))

    def _mergeEnvironment(self, env, expressionList):
        newenv = env.copy()
        for stm in expressionList:
            stm = stm.split('#')[0]
            stm = stm.strip()
            if len(stm) < 1: continue
            mo = regVarSet.match(stm)
            if mo != None:
                rhs = _xe(mo.group(2), newenv)
                rhs = rhs.replace("[PWD]", self.codeRootDir)
                newenv[mo.group(1)] = rhs
            else: print "Invalid ENV expression:", stm

        return newenv

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

    def addCastConfig(self, filename):
        self._storeMru(self.mruCfgCast, filename)
        pass

    def addHostsConfig(self, filename):
        self._storeMru(self.mruCfgHosts, filename)
        pass

    def getOption(self, key):
        if self.options.has_key(key): return self.options[key]
        return ""

    def setOption(self, key, value):
        self.options["%s" % key] =  "%s" % value
