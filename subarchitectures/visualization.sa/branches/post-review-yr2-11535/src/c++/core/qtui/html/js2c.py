#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim

import os, sys
outfile = "v11n_jscode.inc"
infiles = ["formcap.js"]

f = open(outfile, "w")

for fn in infiles:
    fnfull = fn
    lines = open(fnfull).readlines()
    lines = [ "" if ln.strip().startswith("//")
             else ln.rstrip().replace('\\', '\\\\').replace('"', '\\"')
             for ln in lines
            ]
    var = "jscode_" + fn.replace(".", "_").replace("-", "_")
    f.write("// Generated from %s\n" % fn)
    f.write("const QString %s = QString::fromUtf8(\n" % var)
    for ln in lines:
        f.write('   "%s\\n"\n' % ln)
    f.write("   ); // %s\n\n" % var)

f.close()

