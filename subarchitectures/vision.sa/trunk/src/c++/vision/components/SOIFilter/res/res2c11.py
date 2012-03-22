#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim

import os, sys
if len(sys.argv) < 3:
    print "res2c11.py output_file  input_file [input_file ...]"
    sys.exit()

outfile = sys.argv[1]
infiles = sys.argv[2:]
#outfile = "ptuctrl.inc"
#infiles = ["ptucontroller.ui", "ptucontroller.js"]

f = open(outfile, "w")

f.write("// C++11 include file\n")
f.write("// --------------------------------------------------\n\n")

for fn in infiles:
    fnfull = fn
    lines = open(fnfull).readlines()
    lines = [ "" if ln.strip().startswith("//")
             else ln.rstrip()
             for ln in lines
            ]
    var = "res_" + fn.replace(".", "_")
    f.write("// Generated from %s\n" % fn)
    f.write("const char %s[] = R\"((" % var)
    for ln in lines:
        f.write('%s\n' % ln)
    f.write("))\"; // %s\n" % var)
    f.write("// --------------------------------------------------\n\n")

f.close()

