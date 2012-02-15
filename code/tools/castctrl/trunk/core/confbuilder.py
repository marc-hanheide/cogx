#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: oct 2009

# Build configuration files before running various processes.
# Configuration files are copied to a temporary file which
# is then passed to the process.

import os, sys
import re
import logger
import options

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

class CHostMap:
    def __init__(self, localhost=None):
        self._localhost = "127.0.0.1"
        self.clearHosts()
        self.setLocalhost(localhost)

    def clearHosts(self):
        self.hosts = { "localhost": self._localhost, "127.0.0.1": self._localhost }

    def setLocalhost(self, address):
        if address == None or address.strip() == "":
            return
        self._localhost = address
        self.hosts["localhost"] = self._localhost
        self.hosts["127.0.0.1"] = self._localhost

    def isKnownHost(self, hostname):
        return hostname in self.hosts

    def isLocalHost(self, hostname):
        h = hostname
        if h in self.hosts:
            h = self.hosts[hostname]
        if h == "localhost" or h.startswith("127.0.0.") or h == "::1":
            return True
        if self.hosts["localhost"] == h:
            return True
        return False

    def setHostAddress(self, name, addr):
        self.hosts[name] = addr

    def expandHostName(self, name):
        name = self.fixLocalhost(name)
        if name in self.hosts:
            name = self.hosts[name]
        return name

    def fixLocalhost(self, host):
        h = host.lower()
        if h == "localhost" or h.startswith("127.0.0.") or h == "::1":
            if "localhost" in self.hosts: host = self.hosts["localhost"]
        return host

    def items(self):
        return self.hosts.items()

class CCastConfig:
    reInclude = re.compile(r"^\s*include\s+(\S+)", re.IGNORECASE)
    reHostName = re.compile(r"^\s*hostname\s+(\S+)\s+(\S+)", re.IGNORECASE)
    reHost = re.compile(r"^\s*host\s+(\S+)", re.IGNORECASE)
    reSubarch = re.compile(r"^\s*subarchitecture\s+(\S+)", re.IGNORECASE)
    reComponent = re.compile(r"^\s*component\s+(\S+\s+)?(cpp|java|python)\s+(\S+)(\s+.*)?", re.IGNORECASE)
    reSubComponent = re.compile(r"^\s*(\S+\s+)?(cpp|java|python)\s+(\S\S)\s+(\S+)(\s+.*)?", re.IGNORECASE)
    def __init__(self, hostMap):
        # rules: subarch on machine; component on machine; machine file (hosts, components, subarchs)
        # [ (type, token, ip) ]
        #self._localhost = "127.0.0.1"
        #self.hosts = { "localhost": self._localhost, "127.0.0.1": self._localhost }
        self.hostMap = hostMap
        self.rules = []
        self.components = []
        self._cast_hostnames = {} # defined in .cast

    # all but SETVAR, VARDEFAULT and HOSTNAME are 'headers'
    def _isHeader(self, line):
        for rex in [
                CCastConfig.reSubarch, CCastConfig.reHost,
                CCastConfig.reComponent, CCastConfig.reSubComponent]:
            mo = rex.match(line)
            if mo != None: return True
        return False

    def clearRules(self):
        self.rules = []
        #self.hosts = { "localhost": self._localhost, "127.0.0.1": self._localhost }

    #def setLocalhost(self, address):
    #    self._localhost = address
    #    self.hosts["localhost"] = self._localhost
    #    self.hosts["127.0.0.1"] = self._localhost

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
                if rs[1].lower() == "localhost": self.hostMap.setLocalhost(rs[2])
                else: self.hostMap.setHostAddress(rs[1], rs[2])
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
                    # TODO: Should leave hosts as they are; let _update_cast_hostnames take care of translation
                    host = rs[2].strip(" []")
                    if self.hostMap.isKnownHost(host):
                        rs[2] = self.hostMap.expandHostName(host)
                self.rules.append(rs)
                continue
            logger.get().warn("Bad rule: '%s'" % line)

    def removeComment(self, line):
        pos = line.find("#")
        if pos < 0: return line
        if line.find("'") > 0 or line.find('"') > 0:
            # TODO: Handle strings while removing comments!
            pass
        if line.strip().startswith("#EXPAND"):
            return line
        return line.split("#")[0]

    def _cwdRelativeDir(self, fdir):
        reldir = os.path.relpath(fdir)
        if reldir == "" or reldir == ".": fdir = "."
        elif not reldir.startswith("."): fdir = reldir
        return fdir

    def readConfig(self, filename, maxdepth=32):
        if maxdepth < 1:
            raise Exception("Failed to include: '%s' - maxdepth reached." % filename)
        try:
            f = open(filename, "r")
        except Exception as e:
            raise Exception ("Failed to include: '%s'\n%s" % (filename, e))

        fdir = self._cwdRelativeDir(os.path.abspath(os.path.dirname(filename)))
        lines = [
                # CURRENT_DIR is also set by cast
                "SETVAR CURRENT_DIR=%s" % fdir,
                "SETVAR CURRENT_FILE=%s" % os.path.basename(filename),
               ]
        for line in f.readlines():
            line = line.rstrip() # remove EOL
            setting = self.removeComment(line)
            if setting == "": continue
            mo = CCastConfig.reInclude.match(setting)
            if mo != None:
                lines += [
                        "",
                        "# ----- include ------"
                       ]
                fninc = os.path.join(fdir, mo.group(1))
                lines += self.readConfig(fninc, maxdepth-1)
                lines += [
                        "# -----",
                        "",
                        "SETVAR CURRENT_DIR=%s" % fdir,
                        "SETVAR CURRENT_FILE=%s" % os.path.basename(filename),
                       ]
                continue
            lines.append(line)
        return lines


    # done: HOSTNAME entries are overwritten by hconf HOST entries
    # done: new HOSTNAME entries are added
    # maybe: HPAR rules may be dropped (use SETVAR instead?)
    # Regenerate the HOSTNAME entries, put them at the beginning of the file
    def _update_cast_hostnames(self, lines):
        self._cast_hostnames = {}
        ordHosts = []
        removelines = []
        for i,line in enumerate(lines):
            mo = CCastConfig.reHostName.match(line)
            if mo != None:
                self._cast_hostnames[mo.group(1)] = mo.group(2)
                removelines.append(i)
                ordHosts.append(mo.group(1))

        lastremoved = removelines[-1] - len(removelines) + 1 if len(removelines) else 0
        for i in reversed(removelines):
            lines.pop(i)

        # add localhost definition
        if not "localhost" in self._cast_hostnames:
            self._cast_hostnames["localhost"] = self.hostMap.fixLocalhost("localhost")
            ordHosts.append("localhost")

        # add additional hosts from hconf
        for k,v in self.hostMap.items():
            if k in self._cast_hostnames: # update the value
                self._cast_hostnames[k] = v
                continue
            mo = re.match("^[a-zA-Z][a-zA-Z0-9_]*$", k)
            if not mo: continue;
            ordHosts.append(k)
            self._cast_hostnames[k] = v

        # topological sorting of hosts
        # if host uses target, target must be defined first
        def uses(host, target):
            if target == "localhost" and self._cast_hostnames[host] == target:
                return True
            if self._cast_hostnames[host] == "[%s]" % target:
                return True
            return False

        changed = True
        retries = 20
        while changed and retries > 0:
            neword = []
            changed = False
            while len(ordHosts) > 0:
                host = ordHosts.pop(0)
                i = 0
                while i < len(ordHosts):
                    if uses(host, ordHosts[i]):
                        neword.append(ordHosts.pop(i))
                        changed = True
                        continue
                    i += 1
                neword.append(host)
            ordHosts = neword
            retries -= 1

        if retries < 1:
            logger.get().warn("HOSTNAME-s COULD NOT BE SORTED.")

        for h in reversed(ordHosts):
            lines.insert(lastremoved, "HOSTNAME %s %s" % (h, self._cast_hostnames[h]))
        last = len(ordHosts) + lastremoved

        # remove invalid HOST declarations
        defaultHostFound = False
        otherHeaderFound = False
        for i,line in enumerate(lines):
            mo = CCastConfig.reHost.match(line)
            if mo != None:
                host = mo.group(1).strip(" []")
                valid = host in self._cast_hostnames
                if not valid: lines[i] = "# disabled " + line
                if valid and not defaultHostFound: defaultHostFound = not otherHeaderFound
                continue

            if not otherHeaderFound and self._isHeader(line):
                otherHeaderFound = True
                continue

        # add the missing HOST
        if not defaultHostFound:
            lines.insert(last, "HOST localhost")
            last = last + 1


    #  TODO: How to support multiple HOST definitions in .cast?
    #  Remove extra HOSTs if there are rules 
    def prepareConfig(self, filename, afile=CNullFile()):
        """
        Reads the config file and writes it to an open file-like object 'afile'
        using write(). The input file is processed with all the included files.
        Fixes the localchost IP. Distributes the components to remote machines
        according to rules.
        """
        lines = [
            "SETVAR WORKING_DIR=%s" % os.getcwd(),
            "SETVAR CONFIG_DIR=%s" % self._cwdRelativeDir(os.path.dirname(os.path.abspath(filename)))
        ]
        lines += self.readConfig(filename)
        self._update_cast_hostnames(lines)

        removeExtraHosts = len(self.rules) > 0 # ie. a hconf was added
        defaultHostFound = False

        subarch = ""
        for line in lines:
            mo = CCastConfig.reSubarch.match(line)
            if mo != None:
                subarch = mo.group(1)
                disabled = None
                for r in self.rules:
                    if r[0] == "SA" and r[1].lower() == subarch.lower():
                        if len(r) > 3 and not parsebool(r[3], 1):
                            disabled = r
                            break
                if disabled == None: afile.write(line + "\n")
                else: afile.write("# rule(%s) %s\n" % (disabled, line))
                continue

            mo = CCastConfig.reHost.match(line)
            if mo != None:
                if defaultHostFound and removeExtraHosts:
                    # TODO: HOST entries are removed if the server is not defined in _cast_hostnames
                    continue
                host = mo.group(1)
                if subarch == "":
                    for r in self.rules:
                        if r[0] == "SA" and r[1].lower() == "none":
                            host = r[2]
                newline = "HOST %s" % self.hostMap.fixLocalhost(host)
                afile.write(newline + "\n")
                defaultHostFound = True
                continue

            (ok, newline) = self._processComponent(line, subarch)
            if ok:
                afile.write(newline + "\n")
                continue

            afile.write(line + "\n")

    #def _fixLocalhost(self, host):
    #    h = host.lower()
    #    if h == "localhost" or h.startswith("127.0.0.") or h == "::1":
    #        if "localhost" in self.hosts: host = self.hosts["localhost"]
    #    return host

    def _fixHostParam(self, text, param):
        rx = re.compile(r"%s\s+(\"[^\"]+\"|\S+)" % param)
        mo = rx.search(text)
        if mo != None:
            host = self.hostMap.fixLocalhost(mo.group(1))
            text = text[:mo.start(1)] + host + text[mo.end(1):]
        return text

    def _setHostParam(self, text, param, value):
        rx = re.compile(r"%s\s+(\"[^\"]+\"|\S+)" % param)
        mo = rx.search(text)
        if mo != None:
            text = text[:mo.start(1)] + '"' + value + '"' + text[mo.end(1):]
        return text

    def _processComponent(self, line, subarch):
        cpid = None
        mo = CCastConfig.reComponent.match(line)
        if mo != None:
            prefix = "COMPONENT"
            host   = mo.group(1)
            lang   = mo.group(2)
            cptype = ""
            cpid   = mo.group(3)
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

        comp = CComponent(subarch, cpid, cptype, lang)
        self.components.append(comp)

        # host is currently preserved if defined for component in .cast but not in .hconf
        disabled = None
        for r in self.rules:
            if disabled != None: break
            if r[0] == "SA" and r[1].lower() == subarch.lower():
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
        host = self.hostMap.fixLocalhost(host)
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
