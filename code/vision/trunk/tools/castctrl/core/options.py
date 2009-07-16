#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009 

import os, os.path
import re

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
        self.textEditCmd = "gvim --remote %s"

class CCastOptions(object):
    def __init__(self):
        self.mruCfgPlayer = []
        self.mruCfgCast = []
        self.environmentDefault = """
            CAST_DIR=/usr/local
            SA_DIR=[PWD]
            CAST_INSTALL_ROOT=${CAST_DIR}
            CAST_BIN_DIR=${CAST_INSTALL_ROOT}/bin
            CAST_LIB_DIR=${CAST_INSTALL_ROOT}/lib/cast:${SA_DIR}/output/lib
            CAST_CLASSES_DIR=${CAST_INSTALL_ROOT}/share/java/cast.jar
            ICE_CONFIG=${CAST_INSTALL_ROOT}/share/cast/config/cast_ice_config

            LD_LIBRARY_PATH=${CAST_LIB_DIR}:${LD_LIBRARY_PATH}
            DYLD_LIBRARY_PATH=${CAST_LIB_DIR}:${DYLD_LIBRARY_PATH}

            CMD_CPP_SERVER=${CAST_BIN_DIR}/cast-server-c++
            CMD_JAVA_SERVER=java -ea -classpath ${CAST_CLASSES_DIR}:$CLASSPATH cast.server.ComponentServer
            CMD_CAST_CLIENT=java -ea -classpath ${CAST_CLASSES_DIR}:$CLASSPATH cast.clients.CASTClient -f [CAST_CONFIG]
            CMD_PLAYER=player [PLAYER_CONFIG]
            """.split("\n")
        self._environment = None

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
            if l == "[MRU-CAST]": section = self.mruCfgCast
            elif l == "[MRU-PLAYER]": section = self.mruCfgPlayer
            elif l == "[ENVIRONMENT]":
                self._environment = []
                section = self._environment
            elif l.startswith('['): section = None
            elif section != None:
                section.append(ln.rstrip())

    def saveConfig(self, filename):
        f = open(filename, "w")
        f.write("[ENVIRONMENT]\n")
        for ln in self.environment:
            f.write(ln); f.write("\n")
        f.write("[MRU-CAST]\n")
        for ln in self.mruCfgCast:
            f.write(ln); f.write("\n")
        f.write("[MRU-PLAYER]\n")
        for ln in self.mruCfgPlayer:
            f.write(ln); f.write("\n")
        f.close()


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
