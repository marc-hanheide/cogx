#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# @author:  Marko Mahnič
# @created: jun 2009 

import sys, os
__INITIALIZER = None
thisDir = os.path.dirname(__file__)
castPythonDir = os.path.abspath(os.path.join(thisDir, ".."))

class _Initializer:
    def __init__(self):
        pass

    def addSysPath(self, fn):
        try: i = sys.path.index(fn)
        except ValueError:
            # path not in sys.path, so we add it
            sys.path.insert(0, fn)
            print "python: sys.path +=", os.path.basename(fn)

    def addPythonZipLibs(self):
        fp = castPythonDir
        for name in os.listdir(fp):
            if not name.endswith(".pylib"): continue
            self.addSysPath(os.path.join(fp, name))

    def addDistPackages(self):
        fp = os.path.join(castPythonDir, "dist-packages")
        if os.path.exists(fp): self.addSysPath(fp)
        fp = os.path.join(castPythonDir, "dist-slicegen")
        if os.path.exists(fp): self.addSysPath(fp)

if __INITIALIZER == None:
    __INITIALIZER = _Initializer()
    # packages installed as .pylib or in dist-packages will have precedence over
    # packages installed in PYTHON_PATH
    __INITIALIZER.addPythonZipLibs()
    __INITIALIZER.addDistPackages()
