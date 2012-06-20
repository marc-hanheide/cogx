#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# 
# @author: Marko Mahnič
# @created: March 2012
# 

import os, sys
import re

class C03Writer:
    def __init__(self, ofile):
        self.f = ofile
        pass

    def writeVarStart(self, filename, varname):
        self.f.write("// Generated from %s\n" % filename)
        self.f.write("const char %s[] = \n" % varname)

    def writeVarEnd(self, filename, varname):
        self.f.write("   ; // %s\n" % varname)
        self.f.write("// --------------------------------------------------\n\n")

    def writeLine(self, line):
        for c in '\\"':
            line = line.replace(c, '\\' + c)
        self.f.write('   "%s\\n"\n' % line)

class C11Writer:
    def __init__(self, ofile):
        self.f = ofile
        pass

    def writeVarStart(self, filename, varname):
        self.f.write("// Generated from %s\n" % filename)
        self.f.write("const char %s[] = R\"--(" % varname)

    def writeVarEnd(self, filename, varname):
        self.f.write(")--\"; // %s\n" % varname)
        self.f.write("// --------------------------------------------------\n\n")

    def writeLine(self, line):
        self.f.write('%s\n' % line)

c11mode = True
outfile = None
infiles = []
for a in sys.argv[1:]:
    if a == "--c11": c11mode = True
    elif a == "--c03": c11mode = False
    elif outfile == None: outfile = a
    else: infiles.append(a)

if len(infiles) < 1:
    print "res2c11.py [--c11|--c03] output_file  input_file [input_file ...]"
    sys.exit()

f = open(outfile, "w")

if c11mode:
    f.write("// C++11 include file\n")
    writer = C11Writer(f)
else:
    f.write("// C++ include file\n")
    writer = C03Writer(f)
f.write("// --------------------------------------------------\n\n")

for fn in infiles:
    fnfull = fn
    lines = open(fnfull).readlines()
    lines = [ "" if ln.strip().startswith("//")
             else ln.rstrip()
             for ln in lines
            ]
    #var = "res_" + fn.replace(".", "_")
    var = "res_" + re.sub("[^a-zA-Z0-9]+", "_", fn)
    writer.writeVarStart(fn, var)
    for ln in lines:
        writer.writeLine(ln)
    writer.writeVarEnd(fn, var)

f.close()

