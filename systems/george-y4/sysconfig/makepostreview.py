# vim: set fileencoding=utf-8 sw=4 sts=4 ts=8 et :vim
import os, sys, re
import subprocess as subp

branch = "branches/george-y3-postreview"
workroot = os.path.abspath("..")
entries  = ".svn/entries"
rootentries = os.path.join(workroot, entries)
externals = "george-y3.externals"
revision = "18140"

def getRootForExternals():
    lines = open(entries).readlines()
    lines = [ l.strip() for l in lines ]
    count = -1
    for line in lines:
        if line == "dir":
            count = 3
            continue
        count -= 1
        if count == 0:
            return line

def createBranches():
    null = open(os.devnull, "w")
    root = getRootForExternals()
    lines = open(externals).readlines()
    lines = [ l.strip() for l in lines ]
    for i,line in enumerate(lines):
        if not line.startswith("^"): continue
        parts = line.split()[:2]
        repopath = parts[0][2:]
        workpath = parts[1]
        if os.path.basename(repopath) != "trunk":
            continue
        branchpath = os.path.join(os.path.dirname(repopath), branch)
        branchurl = os.path.join(root, branchpath)
        trunkurl = os.path.join(root, repopath)
        print "%02d/%02d Copying: %s" % (i, len(lines), trunkurl)
        try:
            print "Verify branch exists"
            out = subp.check_output(["svn", "info", branchurl], stderr=null)
            print "Branch Exists", branchurl
            continue
        except: pass
        try:
            print "Verify trunk exists"
            out = subp.check_output(["svn", "info", trunkurl], stderr=null)
        except:
            print "Trunk not found", trunkurl
            continue

        try:
            print "svn cp"
            out = subp.check_output(["svn", "cp", "%s@%s" % (trunkurl, revision), branchurl,
                    "-m", "%s: %s" % (branch, repopath)])
        except:
            print " *** Failed to copy", trunkurl

        #print os.path.join(root, repopath)
        #print "   -->", branchpath
        #print " co at", workpath

createBranches()
