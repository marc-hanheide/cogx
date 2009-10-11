#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: april 2009 

import time

class TimerOn:
    def __init__(self):
        self.TIC=None
        self.NTIC=[time.time() for i in xrange(10)]

    def tic(self):
        self.TIC=time.time()

    def toc(self, msg="Elapsed"):
        dt = time.time() - self.TIC
        print msg, dt
        return dt

    def ntic(self, n):
        assert(n >= 0 and n < 10)
        self.NTIC[n]=time.time()

    def ntoc(self, n, msg="Elapsed"):
        assert(n >= 0 and n < 10)
        dt = time.time() - self.NTIC[n]
        print msg, dt
        return dt

class TimerOff:
    def tic(self): pass
    def toc(self, msg="Elapsed"): return 0
    def ntic(self, n): pass
    def ntoc(self, n, msg="Elapsed"): return 0
