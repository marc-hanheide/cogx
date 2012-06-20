#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
import os, sys
here = os.path.abspath(os.path.dirname(__file__))
output = os.path.abspath(here + "/" + ("../" * 7) + "output")
sys.path.insert(0, output + "/python")
sys.path.insert(0, os.path.abspath(here + "/../../dist-packages"))
