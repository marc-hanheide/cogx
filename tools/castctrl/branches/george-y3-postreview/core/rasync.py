#!/usr/bin/python
# vim: set sw=4 ts=8 sts=4 et :vim
import os, sys, stat
import time
import procman

COGX_ROOT=os.getcwd()

# rsync daemon configuration
RSYNC_CONFIG="""
[cogxsource]
   path = %(COGX_ROOT)s
   hosts allow = %(CASTCTRL_HOST)s
   exclude = .svn/ .git/ .bzr/ .hg/ \\
             BUILD/ /Build/ /build/ /logs/ /output/ /bin/ \\
             /tools/v4r/lib/* /tools/v4r/bin/* \\
             /tools/abducer/bin/* \\
             + /CMakeLists.txt \\
             + /build.xml \\
             + */ - /* /*.conf /*.ini /*.sh \\
             *.~* *.*~ *.o *.so
   secrets file = %(PASSWD)s
   # less secure
   use chroot = false
   read only = false
   munge symlinks = false
"""
#  protocol.pb.h protocol.pb.cc
RSYNC_PORT = 10873

# The sender
class CRemoteSync:
    PASSWD = "/tmp/cogxrsync.pass"

    #def __init__(self):
    #    direxclude = ".svn:.hg:.git:.bzr"
    #    direxclude += ":/BUILD:/Build:/build:/output:/logs"
    #    self.direxclude = direxclude

    #    fileexclude = "/*" # We don't sync any normal files from root dir
    #    fileexclude += ":*.~*:*.*~" # backup files
    #    self.fileexclude = fileexclude

    #def getRsyncFilter(self):
    #    opts = []
    #    direxclude = self.direxclude.split(":")
    #    direxclude = [ s + "/" for s in direxclude ]

    #    for d in direxclude:
    #        opts += ["--exclude", d]

    #    fileexclude = self.fileexclude.split(":")
    #    for f in fileexclude:
    #        if f == "/*": # this also excludes all directories, so we re-include them
    #            opts += ["--include", "*/"]
    #        opts += ["--exclude", f]

    #    return opts


    def rsync(self, localhost, remoteProcessManager):
        castAgent = remoteProcessManager.agentProxy
        remoteAddress = remoteProcessManager.address
        if not castAgent.startRsync(localhost):
            return False

        f = open(CRemoteSync.PASSWD, "w")
        f.write("cogxrsync:123123")
        f.close()
        os.chmod(CRemoteSync.PASSWD, stat.S_IRUSR | stat.S_IWUSR)

        try:
            params = {
                "USER": "cogxrsync",
                "PASSWD": CRemoteSync.PASSWD,
                "ROOT": COGX_ROOT,
                "HOST": remoteAddress,
                "PORT": RSYNC_PORT,     # TODO: obtain from agent
            }
            cmd = " ".join(["rsync -r --update --compress",
                # "--dry-run",
                "--delete",
                "--links --safe-links",
                "--verbose",
                "--password-file=%(PASSWD)s",
                "%(ROOT)s/ rsync://%(USER)s@%(HOST)s:%(PORT)d/cogxsource"
               ]) % params
            print cmd
            procman.xrun_wait(cmd)
            # time.sleep(2) # TODO: Do the sync in a thread
        except Exception as e:
            print e
        finally:
            castAgent.stopRsync()

# The receiver
class CRSyncDaemonConfig:
    CONFIG = "/tmp/cogxrsyncd.conf"
    PASSWD = "/tmp/cogxrsyncd.pass"

    def __init__(self):
        self.srcHost = None
        self.port = RSYNC_PORT

    def writeDaemonConfig(self):
        config = RSYNC_CONFIG % ({
            "COGX_ROOT": COGX_ROOT,
            "CASTCTRL_HOST": self.srcHost,
            "PASSWD": CRSyncDaemonConfig.PASSWD,
        })
        f = open(CRSyncDaemonConfig.CONFIG, "w")
        f.write(config)
        f.close()
        f = open(CRSyncDaemonConfig.PASSWD, "w")
        f.write("cogxrsync:123123")
        f.close()
        os.chmod(CRSyncDaemonConfig.PASSWD, stat.S_IRUSR | stat.S_IWUSR)

    def getDaemonCommand(self):
        cmd = "rsync --daemon --no-detach --port=[PORT] --config=[CONFIG]"
        return cmd

    def getDaemonParams(self):
        params={
            "PORT": self.port,
            "CONFIG": CRSyncDaemonConfig.CONFIG,
        }
        return params
