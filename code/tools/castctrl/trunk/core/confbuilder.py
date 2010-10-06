#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: oct 2009

# Build configuration files before running various processes.
# Configuration files are copied to a temporary file which
# is then passed to the process.

import os, sys
import re
import messages
import options

LOGGER = messages.CInternalLogger()

def parsebool(val, default=0):
    val = val.lower().strip()
    if val == "": return default
    if val == "no" or val == "off" or val == "false" or val == "f": return 0
    if val == "yes" or val == "on" or val == "true" or val == "t": return 1
    return int(val)

def quoteParams(parList):
    pars = []
    merging = False
    for p in parList:
        if merging: pars[-1] = pars[-1] + " " + p.strip('"')
        else: pars.append(p.strip('"'))
        if p.startswith('"'): merging = True
        if p.endswith('"'): merging = False
    return pars

# Components extracted from the config file; used for filtering
class CComponent:
    def __init__(self, subarch, cid, ctype, lang):
        self.subarch = subarch
        self.cid = cid
        self.ctype = ctype
        self.lang = lang
        self.status = 0 # included/excluded by filter
        self.enabled = True

    def __cmp__(self, cc):
        def sc(a,b):
            if a < b: return -1
            if a > b: return 1
            return 0
        v = sc(self.cid, cc.cid)
        if v != 0: return v
        v = sc(self.ctype, cc.ctype)
        if v != 0: return v
        return sc(self.lang, cc.lang)

    def __repr__(self):
        return "<CComponent %s>" % self.cid

class CNullFile:
    def write(self, msg): pass

class CCastConfig:
    reInclude = re.compile(r"^\s*include\s+(\S+)", re.IGNORECASE)
    reSubarch = re.compile(r"^\s*subarchitecture\s+(\S+)", re.IGNORECASE)
    reHost = re.compile(r"^\s*host\s+(\S+)", re.IGNORECASE)
    reComponent = re.compile(r"^\s*component\s+(\S+\s+)?(cpp|java|python)\s+(\S+)(\s+.*)?", re.IGNORECASE)
    reSubComponent = re.compile(r"^\s*(\S+\s+)?(cpp|java|python)\s+(\S\S)\s+(\S+)(\s+.*)?", re.IGNORECASE)
    def __init__(self):
        # rules: subarch on machine; component on machine; machine file (hosts, components, subarchs)
        # [ (type, token, ip) ]
        self._localhost = "127.0.0.1"
        self.hosts = { "localhost": self._localhost, "127.0.0.1": self._localhost }
        self.rules = []
        self.subarch = ""
        self.components = []

    def clearRules(self):
        self.rules = []
        self.hosts = { "localhost": self._localhost, "127.0.0.1": self._localhost }

    def setLocalhost(self, address):
        self._localhost = address
        self.hosts["localhost"] = self._localhost
        self.hosts["127.0.0.1"] = self._localhost

    # one rule per line
    #   HOST name address
    #   SA   id   [hostname]/address  # by subarchitecture
    #   ID   id   [hostname]/address  enabled?   # by component id
    #   HPAR   id|--param  [hostname]/address   # changing parameters
    #   TODO?
    #   CO   id   [hostname]/address  enabled?  # by component name
    #   CT   id   [hostname]/address  enabled?  # by component type
    def addRules(self, ruleList):
        for line in ruleList:
            r = line.split("#")[0].strip()
            if r == "": continue
            if r.lower().startswith("host"):
                rs = r.split()
                if rs[1].lower() == "localhost": self.setLocalhost(rs[2])
                else: self.hosts[rs[1].lower()] = rs[2]
                continue
            rs = r.split()
            if len(rs) < 2: continue
            if len(rs) > 3:
                pars = quoteParams(rs[2:])
                rs = rs[:2] + pars
            rs[0] = rs[0].upper()
            goodrule = [ 1 for desc in ['SA', 'ID', 'HPAR'] if rs[0] == desc]
            if len(goodrule) > 0:
                if rs[2].startswith("[") and rs[2].endswith("]"):
                    host = rs[2].strip(" []").lower()
                    if self.hosts.has_key(host): rs[2] = self.hosts[host]
                self.rules.append(rs)
                continue
            LOGGER.warn("Bad rule: '%s'" % line)

    def removeComment(self, line):
        pos = line.find("#")
        if pos < 0: return line
        if line.find("'") > 0 or line.find('"') > 0:
            # TODO: Handle strings while removing comments!
            pass
        return line.split("#")[0]

    def readConfig(self, filename, maxdepth=32):
        if maxdepth < 1:
            return ["# Failed to include: '%s' - maxdepth reached." % filename]
        try:
            f = open(filename, "r")
        except:
            return ["# Failed to include: '%s'" % filename]
        fdir = os.path.abspath(os.path.dirname(filename))
        lines = ["# '%s'" % filename]
        for line in f.readlines():
            line = line.rstrip() # remove EOL
            setting = self.removeComment(line)
            if setting == "": continue
            mo = CCastConfig.reInclude.match(setting)
            if mo != None:
                fninc = os.path.join(fdir, mo.group(1))
                lines += self.readConfig(fninc, maxdepth-1)
                continue
            lines.append(line)
        return lines

    #  TODO: How to support multiple HOST definitions in .cast?
    def prepareConfig(self, filename, afile=CNullFile()):
        """
        Reads the config file and writes it to an open file-like object 'afile'
        using write(). The input file is processed with all the included files.
        Fixes the localchost IP. Distributes the components to remote machines
        according to rules.
        """
        lines = []
        # TODO: Checking for CAST_HAS_SETVAR is temporary; remove when a new latest is published
        # (after 2010-10-06; latest should point to trunk/rev >= 13671)
        opts = options.getCastOptions()
        hasSetVar = opts.xe("$CAST_HAS_SETVAR") == "1"
        if hasSetVar:
            lines += [ "SETVAR CONFIG_DIR=%s" % os.path.dirname(os.path.abspath(filename)) ]
        # ----

        lines += self.readConfig(filename)
        self.subarch = ""
        for line in lines:
            mo = CCastConfig.reSubarch.match(line)
            if mo != None:
                self.subarch = mo.group(1)
                disabled = None
                for r in self.rules:
                    if r[0] == "SA" and r[1].lower() == self.subarch.lower():
                        if len(r) > 3 and not parsebool(r[3], 1):
                            disabled = r
                            break
                if disabled == None: afile.write(line + "\n")
                else: afile.write("# rule(%s) %s\n" % (disabled, line))
                continue

            mo = CCastConfig.reHost.match(line)
            if mo != None:
                host = mo.group(1)
                if self.subarch == "":
                    for r in self.rules:
                        if r[0] == "SA" and r[1].lower() == "none":
                            host = r[2]
                newline = "HOST %s" % self._fixLocalhost(host)
                afile.write(newline + "\n")
                continue

            (ok, newline) = self._processComponent(line)
            if ok:
                afile.write(newline + "\n")
                continue

            afile.write(line + "\n")

    def _fixLocalhost(self, host):
        h = host.lower()
        if h == "localhost" or h == "127.0.0.1":
            if self.hosts.has_key(h): host = self.hosts[h]
        return host

    def _fixHostParam(self, text, param):
        rx = re.compile(r"%s\s+(\"[^\"]+\"|\S+)" % param)
        mo = rx.search(text)
        if mo != None:
            host = self._fixLocalhost(mo.group(1))
            text = text[:mo.start(1)] + host + text[mo.end(1):]
        return text

    def _setHostParam(self, text, param, value):
        rx = re.compile(r"%s\s+(\"[^\"]+\"|\S+)" % param)
        mo = rx.search(text)
        if mo != None:
            text = text[:mo.start(1)] + '"' + value + '"' + text[mo.end(1):]
        return text

    def _processComponent(self, line):
        cpid = None
        mo = CCastConfig.reComponent.match(line)
        if mo != None:
            prefix = "COMPONENT"
            host   = mo.group(1)
            lang   = mo.group(2)
            cpid   = mo.group(3)
            cptype = ""
            rest   = mo.group(4)

        mo = CCastConfig.reSubComponent.match(line)
        if mo != None:
            prefix = ""
            host   = mo.group(1)
            lang   = mo.group(2)
            cptype = mo.group(3)
            cpid   = mo.group(4)
            rest   = mo.group(5)

        if cpid == None: return (0, line)
        if host == None: host = ""
        if rest == None: rest = ""
        rest = rest.strip()

        comp = CComponent(self.subarch, cpid, cptype, lang)
        self.components.append(comp)

        disabled = None
        for r in self.rules:
            if disabled != None: break
            if r[0] == "SA" and r[1].lower() == self.subarch.lower():
                host = r[2]
                if len(r) > 3 and not parsebool(r[3], 1): disabled = r
            elif r[0] == "ID" and r[1].lower() == cpid.lower():
                host = r[2]
                if len(r) > 3 and not parsebool(r[3], 1): disabled = r
            elif r[0] == "HPAR" and rest != "":
                idpar = r[1].split("|")
                if len(idpar) > 1 and (idpar[0].strip() == "" or idpar[0].lower() == cpid.lower()):
                    rest = self._setHostParam(rest, idpar[1], r[2])

        if disabled == None: disabled = ""
        else:
            disabled = "# rule(%s) " % disabled
            comp.enabled = False
        host = self._fixLocalhost(host)
        if rest != "":
            rest = self._fixHostParam(rest, "--player-host") # spatial
            # rest = self._fixHostParam(rest, "--serverHost") # comsys tts

        line = "%s%s %s %s %s %s %s" % (disabled, prefix, host, lang, cptype, cpid, rest)
        return (1, line.strip())


if __name__ == "__main__":
    class XWriter:
        def write(self, text):
            print text.rstrip()

    T = CCastConfig()
    T.setLocalhost("10.0.11.11")
    print "test-include ------------------------------"
    T.prepareConfig("test/confbuilder/test-include.cast", XWriter())
    print "test-rules ------------------------------"
    T.clearRules()
    T.addRules(open("test/confbuilder/test-rules.hconf").readlines())
    T.prepareConfig("test/confbuilder/test-include.cast", XWriter())
