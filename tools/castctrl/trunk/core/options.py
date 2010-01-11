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

def xe(shexpr):
    for x in [regSimple, regSimpleBrace]:
        mos = [mo for mo in x.finditer(shexpr)]
        mos.reverse()
        for m in mos:
            v = ""
            if os.environ.has_key(m.group(1)): v = os.environ[m.group(1)]
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
        self.environmentDefault = [s.lstrip() for s in optdefault.environment.split("\n")]
        self._environment = None
        self.cleanupScript = [s.lstrip() for s in optdefault.cleanup.split("\n")]

    @property
    def environment(self):
        if self._environment == None:
            self._environment = [ln for ln in self.environmentDefault]
        return self._environment

    def loadConfig(self, filename):
        if not os.path.exists(filename): return
        f = open(filename, "r")
        section = None
        for ln in f.readlines():
            l = ln.split('#')[0]
            l = l.strip()
            if l == "[ENVIRONMENT]":
                self._environment = []
                section = self._environment
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
        section = None
        for ln in f.readlines():
            l = ln.split('#')[0]
            l = l.strip()
            if l == "[MRU-CAST]": section = self.mruCfgCast
            elif l == "[MRU-PLAYER]": section = self.mruCfgPlayer
            elif l == "[MRU-HOSTS]": section = self.mruCfgHosts
            elif l.startswith('['): section = None
            elif section != None:
                section.append(ln.rstrip())
        f.close()

    # this should be called only when the config file doesnt exist
    def saveConfig(self, afile):
        f = afile
        f.write("[ENVIRONMENT]\n")
        for ln in self.environment:
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

    def configEnvironment(self):
        # unset all variables
        # THIS IS WRONG!
        #for stm in self.environment:
        #    stm = stm.split('#')[0]
        #    stm = stm.strip()
        #    if len(stm) < 1: continue
        #    mo = regVarSet.match(stm)
        #    if mo != None: os.environ[mo.group(1)] = ""

        curdir = os.path.abspath('.')
        for stm in self.environment:
            stm = stm.split('#')[0]
            stm = stm.strip()
            if len(stm) < 1: continue
            mo = regVarSet.match(stm)
            if mo != None:
                rhs = xe(mo.group(2))
                rhs = rhs.replace("[PWD]", curdir)
                os.environ[mo.group(1)] = rhs
            else: print "Invalid ENV expression:", stm

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
