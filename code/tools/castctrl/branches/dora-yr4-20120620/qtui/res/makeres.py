#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim

import base64

def encodeFile(fname, linesize=80):
    cnt = open(fname, "rb").read()
    enc = base64.standard_b64encode(cnt)
    nb = len(enc) / linesize
    if linesize > 0:
        lines = [enc[i*linesize:(i+1)*linesize] for i in xrange(nb+1)]
    else:
        lines = [enc]
    # print enc == "".join(lines)
    return lines

def addIcon(fname, name, fo):
    lines = encodeFile(fname)
    fo.write('%s = """\\\n' % name)
    fo.write("\n".join(lines))
    fo.write('"""\n\n')

decode_fun = """
import base64
from PyQt4 import QtCore, QtGui
def createPixmap(icon64):
    bin = base64.standard_b64decode(icon64)
    pix = QtGui.QPixmap()
    pix.loadFromData(bin)
    return pix

"""

fo = open("../uiresources.py", "w")
fo.write(decode_fun)
addIcon("cogx_icon.png", "icon_cogx", fo)
fo.close()

