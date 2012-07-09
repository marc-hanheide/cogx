## encoding: utf-8
## Create a matlab deployment tool project file for the cosy vision subarchitecture.
##
## Copyright (c) 2012 Marko Mahnič
##

import sys, glob, os, os.path, fnmatch, getopt
import copy, string

class Options():
    options = [
        ("h", "help", "", "display help"),
        ("P:", "buildroot=", "path", "set the directory of the script"),
        ("M:", "mscriptroot=", "path", "set the directory of the matlab m files"),
        ("B:", "buildoutput=", "path", "set mcc working/output (build) directory"),
        ("L:", "libraryname=", "name", "set the name of the resulting library"),
        ("e:", "export=", "file", "file with a list of required m-files from source tree"),
        ("a:", "export-all", "", "export all m-files from source tree"),
        ("", "cppexport=", "file", "file with a list of required C/CPP files from source tree"),
        ("", "separatectf=", "bool", "true to create separate CTF, false to embed in DLL/SO"),
        ("", "makefile", "", "generate a makefile instead of a shell script"),
    ]
    def __init__(self, argv):
        self.program = os.path.split(argv[0])[1]
        self.prjroot = os.path.abspath(".")
        self.sourceroot = os.path.abspath("../..")
        if os.name == "posix": self.prjname = "libcosyvision"
        else: self.prjname = "CosyVision"
        self.matlabversion = 750  ## 2007a
        self.matlabdir = None
        self.builddir = None
        self.distribdir = None ## only for deployment tool
        self.exportlist = None
        self.cppexportlist = None
        self.exportall = False
        self.format = "sh"

        self.ParseOptions(argv)
        
        if self.matlabdir == None:
            self.matlabdir = "$(MATLABROOT)" ## requires 2007b / 751
            if self.matlabversion < 751:
                if os.name == "posix": self.matlabdir = "/opt/matlab"
                else: self.matlabdir = "c:\\Program Files\\Matlab"
        if self.builddir == None: self.builddir = os.path.normpath(os.path.join(self.prjroot, "xdata/build"))
        if self.distribdir == None: self.distribdir = os.path.normpath(os.path.join(self.prjroot, "xdata/distrib"))
        if self.exportlist == None and not self.exportall:
            el = "mfile-export.txt"
            print "%s: Export list not specified. Using '%s'" % (self.program, el)
            self.exportlist = os.path.abspath(el)
    
    def GetGetoptOptions(self):
        shortopt = string.join([opt[0] for opt in Options.options], "")
        longopt = [opt[1] for opt in Options.options if opt[1] != ""]
        return shortopt, longopt
    
    def ParseOptions(self, argv):
        shortopt, longopt = self.GetGetoptOptions()
        try:
            opts, args = getopt.getopt(argv[1:], shortopt, longopt)
            for opt, arg in opts:
                if opt in ("-h", "--help"):
                    self.Usage()
                    sys.exit()
                elif opt in ("-M", "--mscriptroot"):
                    self.sourceroot = os.path.abspath(arg)
                elif opt in ("-P", "--buildroot"):
                    self.prjroot = os.path.abspath(arg)
                elif opt in ("-L", "--libraryname"):
                    self.prjname = arg
                elif opt in ("-B", "--buildoutput"):
                    self.builddir = os.path.abspath(arg)
                elif opt in ("-e", "--export"):
                    self.exportlist = os.path.abspath(arg)
                elif opt in ("-a", "--export-all"):
                    self.exportall = True
                elif opt in ("--cppexport"):
                    self.cppexportlist = os.path.abspath(arg)
                elif opt in ("--separatectf"):
                    self.separatectf = arg
                elif opt in ("--makefile"):
                    self.format = "make"
                else:
                    print "Unhandled option %s" % opt
                    self.Usage()
                    sys.exit()
        except getopt.GetoptError:
            self.Usage()
            sys.exit(2)

    def Usage(self):
        shortopt = [opt[0] for opt in Options.options]
        print """%s -%s""" % (self.program, string.join(shortopt, " -"))
        optlines = []
        optlen = 0
        for opt in Options.options:
            if len(opt[1]) < 2: optdisp = "-%s" % opt[0].replace(":", " %s" % opt[2])
            else: optdisp = "-%s, --%s" % (opt[0][:1], opt[1].replace("=", "=%s" % opt[2]))
            optlines.append((optdisp, opt[3]))
            if len(optdisp) > optlen: optlen = len(optdisp)
            
        for line in optlines:
            print "   %s %s" % (line[0].ljust(optlen+2), line[1])

opt = None
otherfilemasks = ["*.bmp", "*.fig", "*.mat", "*.txt"]

def FindProjectDirs(rootdir):
    res = []
    prunedirs = ["*/.svn", "*/backup", "*/demo", "*/xdata", "*/prjdeploy", "*/attic"]
    skippaths = []
    for dn in prunedirs:
        sp = os.path.join(rootdir, dn)
        skippaths.append(os.path.normpath(sp))
        if not dn.endswith("*"): skippaths.append(os.path.normpath("%s/*" % sp))
        if dn.startswith("*/"):
            sp = os.path.join(rootdir, dn[2:])
            skippaths.append(os.path.normpath(sp))
            if not dn.endswith("*"): skippaths.append(os.path.normpath("%s/*" % sp))
    for root, dirs, files in os.walk(rootdir, topdown=True):
        bad = False
        for sp in skippaths:
            if fnmatch.fnmatch(root, sp):
                bad = True
                break
        if not bad: res.append(root)
    return res

def FindExportedFunctions(dirlist):
    global opt
    res = []
    if opt.exportall:
        pass
    elif opt.exportlist != None:
        funclist = [f.strip() for f in open(opt.exportlist, "r").readlines()]
        funclist = [f for f in funclist if len(f) > 0 and f[0] != "#"]
        for func in funclist:
            for dir in dirlist:
                path = os.path.join(dir, func + ".m")
                if os.path.exists(path):
                    res.append(path)
                    break # dir
    return res

# used for dependencies in makefile
def FindMatlabFiles(dirlist):
    global opt
    res = []
    for d in dirlist:
        files = os.listdir(d)
        for f in files:
            if f.endswith(".m") or f.endswith(".fig"):
                res.append(os.path.join(d, f))
    return res

def FindExportedCppFiles(dirlist):
    global opt
    res = []
    if opt.cppexportlist != None:
        funclist = [f.strip() for f in open(opt.cppexportlist, "r").readlines()]
        funclist = [f for f in funclist if len(f) > 0 and f[0] != "#"]
        for fn in funclist:
            for dir in dirlist:
                path = os.path.join(dir, fn)
                if os.path.exists(path):
                    res.append(path)
                    break # dir
    return res

def FindOtherFiles():
    pass

def main(argv):
    global opt
    opt = Options(argv)
    
    projectdirs = FindProjectDirs(opt.sourceroot)
    exportfiles = FindExportedFunctions(projectdirs)
    cppexportfiles = FindExportedCppFiles(projectdirs)
    allfiles = FindMatlabFiles(projectdirs)
    
    if opt.format == "sh":
        doctemplate = open("mccbuild.sh.in", "r").read()
        joiner = "\n"
        outfile = "build-%s.sh" % opt.prjname
    elif opt.format == "make":
        doctemplate = open("mccbuild.mak.in", "r").read()
        joiner = " \\\n  "
        outfile = "%s.mak" % opt.prjname

    ## OUTPUT
    ## (the directories can also be added by matlab when the project is 
    ## loaded in the deployment tool for the first time)
    alldirs = []
    alldirs.extend(copy.copy(projectdirs))
    alldirs = [ " -I %s" % os.path.normpath(d)
          for d in alldirs ]

    exportfiles = [ " %s" % os.path.normpath(f)
          for f in exportfiles ]

    cppexportfiles = [ " %s" % os.path.normpath(f)
          for f in cppexportfiles ]

    dict = {
        "prjname": opt.prjname,
        "prjroot": opt.prjroot,
        "sourceroot": opt.sourceroot,
        "builddir": opt.builddir,
        "distribdir": opt.distribdir,
        "includedirs": string.join(alldirs, joiner),
        "mfiles": string.join(exportfiles, joiner),
        "cppfiles": string.join(cppexportfiles, joiner),
        "allfiles": string.join(allfiles, joiner),
        "otherfiles": "",
        "separatectf": opt.separatectf,
    }

    fout = open (os.path.join(opt.prjroot, outfile), "w")
    fout.write(doctemplate % dict)
    fout.close()

if __name__ == "__main__":
    main(sys.argv)
