# vim: set ft=python ts=8 sw=4 et :vim #

from properties import CPropertySet

class CServerInfo(CPropertySet):
    def __init__(self, name, **kwargs):
        super(CServerInfo, self).__init__(name, **kwargs)
        self.enabled = True
        self.command = None
        self.workdir = None
        self.defaultVars = []

        if 'group' in kwargs: self.group = kwargs['group']
        else: self.group = 'A'

    def setVar(self, name, value):
        self.defaultVars.append( (name, value) )

    def _valid_lines(self, strValue):
        lines = [l.strip() for l in strValue.split("\n") if l.strip() != ""]
        lines = [l for l in lines if not l.startswith("#")]
        return lines

    def setPathList(self, name, value):
        self.setVar(name, ":".join(self._valid_lines(value)))

    def setCommand(self, cmd, workdir=None):
        self.command = " ".join(self._valid_lines(cmd))
        self.workdir = workdir

    def getParameters(self):
        params = {}
        for p in self.properties:
            params[p.name] = "%s" % p.value
        return params if len(params) > 0 else None

    def getEnvVarScript(self):
        if len(self.defaultVars) < 1: return None
        script = "\n".join(["%s=%s" % (v[0], v[1]) for v in self.defaultVars])
        return script

class CServerManager:
    def __init__(self):
        self.servers = []

    def addServersFromFile(self, filename):
        self.servers = self.servers + self.discoverServers(filename)

    # Load config files to discover servers
    def discoverServers(self, filename):
        srvrs = []

        def Server(name, **kwargs):
            csi = CServerInfo(name, **kwargs)
            srvrs.append(csi)
            return csi

        glvars = { 'Server': Server }
        try:
            exec open(filename).read() in glvars
        except Exception as e:
            print e

        return srvrs

    def saveServerConfig(self, file):
        for csi in self.servers:
            file.write("\n### %s ###\n" % (csi.name))
            file.write("[Server:%s]\n" % (csi.name.upper()))
            file.write("enabled=%d\n" % (1 if csi.enabled else 0))
            for prop in csi.properties:
                v = prop.value
                if v == None: v = ""
                file.write("%s=%s\n" % (prop.name, v))

            for prop in csi.properties:
                if not prop.mruEnabled: continue
                if not prop.mruHistory: continue
                if len(prop.mruHistory) < 1: continue

                file.write("\n[MRU:%s-%s]\n" % (csi.name.upper(), prop.name))
                for i,h in enumerate(prop.mruHistory):
                    file.write("%03d=%s\n" % (i+1, h))

    def loadServerConfig(self, filename):
        import ConfigParser
        p = ConfigParser.RawConfigParser()
        p.read(filename)
        for csi in self.servers:
            section = "Server:%s" % csi.name.upper()
            if not p.has_section(section): continue
            option = "enabled"
            if p.has_option(section, option):
                csi.enabled = p.getint(section, option)
            for prop in csi.properties:
                option = "%s" % prop.name
                if not p.has_option(section, option): continue
                prop.value = p.get(section, option)
                prop.mruHistory = None

            for prop in csi.properties:
                if not prop.mruEnabled: continue
                section = "MRU:%s-%s" % (csi.name.upper(), prop.name)
                if not p.has_section(section): continue
                items = sorted(p.items(section))
                if len(items) < 1: continue
                prop.mruHistory = [ v[1] for v in items ]


