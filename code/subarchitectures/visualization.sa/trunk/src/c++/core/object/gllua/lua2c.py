#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim

import os, sys
outfile = "v11n_luacode.inc"
infiles = ["displist.lua"]

f = open(outfile, "w")

for fn in infiles:
    fnfull = fn # os.path.join(indir, fn)
    lines = open(fnfull).readlines()
    lines = [ "" if ln.strip().startswith("--")
             else ln.rstrip().replace('\\', '\\\\').replace('"', '\\"')
             for ln in lines
             # if ln.strip() != ""
            ]
    var = "luacode_" + fn.replace(".", "_")
    f.write("// Generated from %s\n" % fn)
    f.write("const char %s[] =\n" % var)
    for ln in lines:
        f.write('   "%s\\n"\n' % ln)
    f.write("   ; // %s\n\n" % var)

f.close()

