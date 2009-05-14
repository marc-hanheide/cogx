#! /usr/bin/env python
# -*- coding: latin-1 -*-

import os.path, os, re

def replaced(re_obj, fpath):
    new_lines = []
    change = False
    for line in open(fpath):
        new_line = re_obj.sub(new_name, line)
        new_lines.append(new_line)
        if new_line != line:
            change = True
    if change:
        return new_lines

def visit(arg, dirname, names):
    if ".svn" in dirname:
        return
    domain, re_obj, new_name = arg
    for fn in names:
        if not fn.endswith(".mapl"):
            continue
        new_fn = re_obj.sub(new_name, fn)
        fpath = os.path.join(dirname, fn)
        new_lines = replaced(re_obj, fpath)
        if new_lines:
            outpath = os.path.join(dirname, new_fn)
            print "found changes in %s. Writing to %s" % (fpath, outpath)
            open(outpath, "w").writelines(new_lines)
            if new_fn != fn:
                os.remove(fpath)
            

def main(domain, old_name, new_name):
    re_obj = re.compile(old_name, re.IGNORECASE)
    if domain:
        os.chdir(domain)
    os.path.walk(".", visit, (domain, re_obj, new_name))
    
    
if __name__ == "__main__":
    import sys
    domain = ""
    try:
        old_name, new_name = sys.argv[1:3]
        if len(sys.argv) == 4:
            domain = sys.argv[3]
    except:
        print "Usage: %s old_name new_name [domain ]" % sys.argv[0]
        print "Either call this is in a specific scenario directory with 2 args or"
        print "in the domains/ directory with the domain name as a 3rd arg"
        sys.exit()
    main(domain, old_name, new_name)
