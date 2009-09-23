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
        self.textEditCmd = "gvim --servername CAST --remote %s"

class CCastOptions(object):
    def __init__(self):
        self.mruCfgPlayer = []
        self.mruCfgCast = []
        env = """
            COGX_ROOT=[PWD]
            COGX_BUILD_DIR=${COGX_ROOT}/BUILD
            COGX_LIB_DIR=${COGX_ROOT}/output/lib
            COGX_PY_DIR=${COGX_ROOT}/output/python

            CAST_DIR=/usr/local
            CAST_INSTALL_ROOT=${CAST_DIR}

            CAST_BIN_PREFIX=bin
            CAST_BIN_DIR=${CAST_INSTALL_ROOT}/${CAST_BIN_PREFIX}

            CAST_LIB_PREFIX=lib/cast
            CAST_LIB_DIR=${CAST_INSTALL_ROOT}/${CAST_LIB_PREFIX}
            CAST_PY_DIR=${CAST_LIB_DIR}/python

            CAST_JAR=${CAST_INSTALL_ROOT}/share/java/cast.jar

            CAST_CONFIG_PATH=share/cast/config/cast_ice_config
            CAST_ICE_CONFIG=${CAST_INSTALL_ROOT}/${CAST_CONFIG_PATH}

            ICE_CONFIG=${CAST_ICE_CONFIG}
            ICE_JARS=/usr/share/java/Ice.jar:/usr/share/java/ant-ice.jar

            CURE_LIB_PATH=/home/cogx/svn/cosycure/lib/cure
            CUDA_LIB_PATH=/usr/local/cuda/lib

            PATH=${COGX_ROOT}/output/bin:${PATH}
            LD_LIBRARY_PATH=${CAST_LIB_DIR}:${COGX_LIB_DIR}:${CURE_LIB_PATH}:${CUDA_LIB_PATH}:${LD_LIBRARY_PATH}
            DYLD_LIBRARY_PATH=${CAST_LIB_DIR}:${COGX_LIB_DIR}:${CURE_LIB_PATH}:${CUDA_LIB_PATH}:${DYLD_LIBRARY_PATH}

            CLASSPATH=${CLASSPATH}:${ICE_JARS}:${CAST_JAR}
            CMD_JAVA=java -ea -classpath ${CLASSPATH}

            PYTHONPATH=${PYTHONPATH}:${CAST_PY_DIR}:${COGX_PY_DIR}

            CMD_CPP_SERVER=${CAST_BIN_DIR}/cast-server-c++
            CMD_JAVA_SERVER=${CMD_JAVA} cast.server.ComponentServer
            CMD_PYTHON_SERVER=python -m ComponentServer
            CMD_CAST_CLIENT=${CMD_JAVA} cast.clients.CASTClient -f [CAST_CONFIG]
            CMD_PLAYER=player [PLAYER_CONFIG]
            CMD_PEEKABOT=peekabot
            """.split("\n")
        self.environmentDefault = [s.lstrip() for s in env]
        self._environment = None
        self.cleanupScript = []

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
            elif l.startswith('['): section = None
            elif section != None:
                section.append(ln.rstrip())
        f.close()

    def saveConfig(self, afile):
        f = afile
        f.write("[ENVIRONMENT]\n")
        for ln in self.environment:
            f.write(ln); f.write("\n")
        f.write("[CLEANUP-SCRIPT]\n")
        for ln in self.cleanupScript:
            f.write(ln); f.write("\n")

    def saveHistory(self, afile):
        f = afile
        f.write("[MRU-CAST]\n")
        for ln in self.mruCfgCast:
            f.write(ln); f.write("\n")
        f.write("[MRU-PLAYER]\n")
        for ln in self.mruCfgPlayer:
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
