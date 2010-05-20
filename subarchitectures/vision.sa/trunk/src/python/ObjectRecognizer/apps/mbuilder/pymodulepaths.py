#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
import os, sys
output = os.path.abspath("../" * 7 + "output")
sys.path.insert(0, output + "/python")
sys.path.insert(0, os.path.abspath("../../dist-packages"))
