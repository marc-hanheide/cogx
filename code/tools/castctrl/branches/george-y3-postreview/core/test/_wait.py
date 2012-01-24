#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim

import sys, time, random

if len(sys.argv) > 1: myid = sys.argv[1]
else: myid = sys.argv[0]

while True:
    print myid, "-->", time.ctime()
    time.sleep((1000 + random.randint(0, 400)) / 1000.0)

