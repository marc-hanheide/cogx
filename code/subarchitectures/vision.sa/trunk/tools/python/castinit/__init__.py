#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:
# @author:  Marko Mahnič
# @created: jun 2009 

import sys, os, os.path
__INITIALIZER = None

class __Initializer:
    def __init__(self):
        pass

    def addSysPath(self, fn):
        try: i = sys.path.index(fn)
        except ValueError:
            sys.path.insert(0, fn)
            print fn

    def addPythonZipLibs(self):
        # TODO: Should python_path be used?
        try: v = os.environ["SA_DIR"]
        except: v = "."
        fp = os.path.abspath("%s/output/python" % v)
        for name in os.listdir(fp):
            if not name.endswith(".pylib"): continue
            self.addSysPath(os.path.join(fp, name))

if __INITIALIZER == None:
    __INITIALIZER = __Initializer()
    __INITIALIZER.addPythonZipLibs()
