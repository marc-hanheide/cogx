#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author: Marko Mahnič
# Created: June 2009

class CHostDef:
    def __init__(self, name, address):
        self.name = name
        self.address = address # IP or [reference]
        self.resolved = None
        if not self.isHostRef():
            self.resolved = self.address

    def setAddress(self, address):
        self.address = address # IP or [reference]
        if self.isHostRef():
            self.resolved = None
        else:
            self.resolved = self.address

    def __repr__(self):
        return "<CHostDef %s(%s)>" % (self.name, self.resolved)

    def isLocalHostName(self):
        return self.name == "localhost" or self.name.startswith("127.0.0.") or self.name == "::1"

    def isHostRef(self):
        if self.name == "localhost": return True
        return self.name.startswith("[") and self.name.endswith("]")

    def getHostRef(self):
        if not self.isHostRef(): return ""
        return self.address.strip("[]")

    def references(self, otherHost):
        if self.address == "localhost":
            return self.address == otherHost
        return self.address == "[%s]" % otherHost

# HOST map defined in .hconf
class CHostMap:
    def __init__(self, localhost=None):
        self.hosts = {}
        self._localhost = "127.0.0.1"
        self.clearHosts()
        self.setLocalhost(localhost)

    def setHost(self, name, address):
        self.hosts[name] = CHostDef(name, address)

    def setHostAddress(self, name, address):
        self.hosts[name] = CHostDef(name, address)

    def clearHosts(self):
        self.setHost("localhost", self._localhost)
        self.setHost("127.0.0.1", self._localhost)

    def setLocalhost(self, address):
        if address == None or address.strip() == "":
            return
        self._localhost = address
        for h in self.hosts.values():
            if h.isLocalHostName():
                h.setAddress(self._localhost)

    def isKnownHost(self, hostname):
        return hostname in self.hosts

    def isLocalHost(self, hostname):
        h = hostname
        if h in self.hosts:
            h = self.hosts[hostname].address
        if h == "localhost" or h.startswith("127.0.0.") or h == "::1":
            return True
        if self.hosts["localhost"].address == h:
            return True
        return False

    def expandHostName(self, name):
        name = self.fixLocalhost(name)
        rslvd = self._resolve(name, self.hosts)
        if rslvd == None:
            if name in self.hosts:
                rlsvd = self.hosts[name].address
            else: rslvd = name
        return rslvd

    def fixLocalhost(self, host):
        h = host.lower()
        if h == "localhost" or h.startswith("127.0.0.") or h == "::1":
            if "localhost" in self.hosts: host = self.hosts["localhost"].address
        return host

    def items(self):
        return self.hosts.items()

    def _isHostRef(self, name):
        return name.startswith("[") and name.endswith("]")

    def _resolve(self, name, hosts, tried={}):
        if not name in hosts:
            return None
        if name in tried:
            return None
        host = hosts[name]
        if host.isHostRef():
            tried[name] = 1
            return self._resolve(name, hosts, tried)
        return host.resolved

    def resolveKnownHosts(self):
        for h in self.hosts.values():
            h.resolved = self._resolve(h.name, self.hosts)

    # @param defaultHosts is an ordered list of pairs [host, mapped-to]
    # @returns an ordered list of triplets [host, mapped-to, effective]
    def getResolvedHosts(self, defaultHosts):
        hosts = []
        hostDict = {}
        for h in defaultHosts:
            hostDict[h.name] = h
            hosts.append(h)

        # add additional hosts from hconf
        for k,h in self.hosts.items():
            if k in hostDict: # update the value
                hostDict[k] = h
                continue
            mo = re.match("^[a-zA-Z][a-zA-Z0-9_]*$", k)
            if not mo: continue;
            hosts.append(h)
            hostDict[k] = h

        # topological sorting of hosts
        # if host refereces target, target must be defined first
        changed = True
        retries = 20
        while changed and retries > 0:
            neword = []
            changed = False
            while len(hosts) > 0:
                host = hosts.pop(0)
                i = 0
                while i < len(hosts):
                    if host.references(hosts[i].name):
                        neword.append(hosts.pop(i))
                        changed = True
                        continue
                    i += 1
                neword.append(host)
            hosts = neword
            retries -= 1

        if retries < 1:
            logger.get().warn("HOSTNAME-s COULD NOT BE SORTED.")

        for h in hosts:
            h.resolved = self._resolve(h.name, hostDict)
        return hosts

