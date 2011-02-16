#! /usr/bin/env python
# -*- coding: latin-1 -*-

"""\
:Author: M. Simionato / M. Brenner
:Date: April 2004 / July 2006
:Title: A much simplified interface to optparse.

You should use optionparse in your scripts as follows.
First, write a module level docstring containing something like this
(this is just an example):

'''usage: %prog [options] some_arg another_arg [optional_arg1 [opt_arg2]]
   -d, --delete: delete all files
   -e, --erase = ERASE: erase the given file'''
   
Then write a main program of this kind:

# sketch of a script to delete files
if __name__=='__main__':
    import optionparse
    option,args=optionparse.parse(__doc__)
    if option.delete: print "Delete all files"
    elif option.erase: print "Delete the given file"

Notice that ``myoptionparse`` parses the docstring by looking at the
characters ",", ":", "=", "\\n", so be careful in using them. If
the docstring is not correctly formatted you will get a SyntaxError
or worse, the script will not work as expected.

Note that all named positional arguments, i.e. all posisitonal arguments that
are mentioned in the the usage line, will be stored under their corresponding name
in the first element of the return tuple; only additional args are returned
as a list in the second tuple element.
"""

import optparse, re, sys

USAGE = re.compile(r'(?s)\s*usage: (.*?)(\n[ \t]*\n|$)')

def nonzero(self): # will become the nonzero method of optparse.Values       
    "True if options were given"
    for v in self.__dict__.itervalues():
        if v is not None: return True
    return False

optparse.Values.__nonzero__ = nonzero # dynamically fix optparse.Values

class ParsingError(Exception): pass

def get_argnames(argline):
    argline = argline.replace("[options]", "")
    argline = argline.replace("[...]", "")
    try:
        optstart = argline.index("[")
    except ValueError:
        optstart = len(argline)
    names = argline[0:optstart].split()[1:]
    optnames = argline[optstart:].replace("[", "").replace("]", "").split()
    return names, optnames        

def convert2num(v):
    val = v
    try:
        val = float(v)
        val  = int(v)
    except:
        pass
    return val
    
class OptionParser(optparse.OptionParser):
    def __init__(self, *args, **kwargs):
        if "from_doc" in kwargs:
            docstring = kwargs["from_doc"]
            del kwargs["from_doc"]
        optparse.OptionParser.__init__(self, *args, **kwargs)
        self.setup_from_doc(docstring)
        
    def error(self, msg):
        """error(msg : string)

        Print a usage message incorporating 'msg' to stderr and exit.
        If you override this in a subclass, it should not return -- it
        should either exit or raise an exception.
        """
        self.print_help(sys.stderr)
        self.exit(2, "\n%s error: %s\n\n" % (self.get_prog_name(), msg))

    def setup_from_doc(self, docstring):
        match = USAGE.search(docstring)
        if not match: raise ParsingError("Cannot find the option string")
        optlines = match.group(1).splitlines()
        self.argnames, self.optional_argnames = get_argnames(optlines[0])
        try:
            self.set_usage(optlines[0])
            for line in optlines[1:]:
                opt, help = line.split(':', 1)[:2]
                short, long = "", ""
                try:
                    short, long = opt.split(',', 1)[:2]
                except ValueError:
                    if "--" in opt:
                        long = opt
                    else:
                        short = opt
                if '=' in opt:
                    action = 'store'
                    long = long.split('=')[0]
                else:
                    action = 'store_true'
                self.add_option(short.strip(), long.strip(),
                             action = action, help = help.strip())
        except (IndexError,ValueError):
            raise ParsingError("Cannot parse the option string correctly")

    def set_defaults_from(self, adict, prefix=""):
        "look for appropriate default values in adict"
        for k in self.defaults:
            v = adict.get(k)
            if v is not None:
                self.defaults[k] = v
         
    def parse_args(self, args=None, values=None):
        options, args = optparse.OptionParser.parse_args(self, args, values)
        # check positional arguments
        minargs = len(self.argnames)
        maxargs = minargs + len(self.optional_argnames)
        if minargs > len(args):
            if self.optional_argnames:
                self.error("%s requires %d-%d arguments, not %d." % (sys.argv[0], minargs, maxargs, len(args)))
            self.error("%s requires exactly %d mandatory arguments, not %d." % (sys.argv[0], minargs, len(args)))
        names = self.argnames+self.optional_argnames
        for name in names:
            options.__dict__[name] = None  # make name known
        options.__dict__.update(dict(zip(names, args)))
        unnamed_args = []
        if len(args) - maxargs > 0:
            unnamed_args = args[maxargs:]
        # try to adjust types of option values
        unnamed_args = [convert2num(v) for v in unnamed_args]
        for k, v in options.__dict__.items():
            options.__dict__[k]  = convert2num(v)
        return options, unnamed_args
