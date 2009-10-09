## encoding: utf-8
## Create a matlab deployment tool project file for the cosy vision subarchitecture.
##
## Copyright (c) 2008 Marko Mahnič
##

import sys, glob, os, os.path, fnmatch, getopt
import copy, string

class Options():
    options = [
        ("h", "help", "", "display help"),
        ("V:", "matlabversion=", "num", "set matlab version (eg. 751 for v7.5 2007b)"),
        ("R:", "matlabroot=", "path", "set matlab root"),
        ("P:", "projectroot=", "path", "set the directory of the project file"),
        ("M:", "mscriptroot=", "path", "set the directory of the matlab m files"),
        ("B:", "buildoutput=", "path", "set mcc working/output directory"),
        ("D:", "distriboutput=", "path", "set directrory for distribution files"),
        ("L:", "libraryname=", "name", "set the name of the resulting library"),
        ("e:", "export=", "file", "file with a list of required m-files from source tree"),
        ("a:", "export-all", "", "export all m-files from source tree"),
        ("", "cppexport=", "file", "file with a list of required C/CPP files from source tree"),
    ]
    def __init__(self, argv):
        self.program = os.path.split(argv[0])[1]
        self.prjroot = os.path.abspath(".")
        self.cosyvisionmatlab = os.path.abspath("..")
        if os.name == "posix": self.prjname = "libcosyvision"
        else: self.prjname = "CosyVision"
        self.matlabversion = 750  ## 2007a
        self.matlabdir = None
        self.builddir = None
        self.distribdir = None ## only for deployment tool
        self.exportlist = None
        self.cppexportlist = None
        self.exportall = False

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
                elif opt in ("-R", "--matlabroot"):
                    self.matlabdir = arg
                elif opt in ("-V", "--matlabversion"):
                    self.matlabversion = int(arg)
                elif opt in ("-M", "--mscriptroot"):
                    self.cosyvisionmatlab = os.path.abspath(arg)
                #~ elif opt in ("-H", "--headeroutput"):
                    #~ self.headeroutputdir = arg
                elif opt in ("-P", "--projectroot"):
                    self.prjroot = os.path.abspath(arg)
                elif opt in ("-L", "--libraryname"):
                    self.prjname = arg
                elif opt in ("-B", "--buildoutput"):
                    self.builddir = os.path.abspath(arg)
                elif opt in ("-D", "--distriboutput"):
                    self.distribdir = os.path.abspath(arg)
                elif opt in ("-e", "--export"):
                    self.exportlist = os.path.abspath(arg)
                elif opt in ("-a", "--export-all"):
                    self.exportall = True
                elif opt in ("--cppexport"):
                    self.cppexportlist = os.path.abspath(arg)
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

def FindToolboxDirs(rootdir):
    dirstemplate = open("matlab.dir.in", "r").read().replace("/", os.sep)
    res = []
    d = { "MATLABROOT": os.path.normpath(rootdir) }
    res = (dirstemplate % d).splitlines()
    return res

def FindProjectDirs(rootdir):
    res = []
    prunedirs = ["*/.svn", "*/backup", "*/demo", "*/xdata", "*/prjdeploy"]
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
    
    projectdirs = FindProjectDirs(opt.cosyvisionmatlab)
    exportfiles = FindExportedFunctions(projectdirs)
    cppexportfiles = FindExportedCppFiles(projectdirs)
    
    doctemplate = open("deployment.prj.in", "r").read()

    ## OUTPUT
    ## (the directories can also be added by matlab when the project is 
    ## loaded in the deployment tool for the first time)
    toolboxdirs = FindToolboxDirs(opt.matlabdir)
    alldirs = copy.copy(toolboxdirs)
    alldirs.extend(copy.copy(projectdirs))
    for i in range(len(alldirs)): 
        alldirs[i] = "    <Directory>%s</Directory>" % os.path.normpath(alldirs[i])

    for i in range(len(exportfiles)): 
        exportfiles[i] = "        <file>%s</file>" % os.path.normpath(exportfiles[i])

    for i in range(len(cppexportfiles)): 
        cppexportfiles[i] = "        <file>%s</file>" % os.path.normpath(cppexportfiles[i])

    dict = {
        "prjname": opt.prjname,
        "prjroot": opt.prjroot,
        "builddir": opt.builddir,
        "distribdir": opt.distribdir,
        "directories": string.join(alldirs, "\n"),
        "mfiles": string.join(exportfiles, "\n"),
        "cppfiles": string.join(cppexportfiles, "\n"),
        "otherfiles": ""
    }

    fout = open (os.path.join(opt.prjroot, "%s.prj" % opt.prjname), "w")
    fout.write(doctemplate % dict)
    fout.close()

if __name__ == "__main__":
    main(sys.argv)
