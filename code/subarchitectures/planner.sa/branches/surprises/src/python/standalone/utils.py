#! /usr/bin/env python
# -*- coding: latin-1 -*-

import re
import copy
import os
from itertools import *
import inspect
import time
from contextlib import contextmanager

from subprocess import PIPE, STDOUT

class Struct:
    """Create an instance with argument=value slots.
    This is for making a lightweight object whose class doesn't matter."""
    def __init__(self, **entries):
        self.__dict__.update(entries)

    def merge(self, other):
        for key, value in other.__dict__.iteritems():
            if isinstance(value, Struct) and isinstance(self.__dict__.get(key, None), Struct):
                self.__dict__[key].merge(value)
            else:
                self.__dict__[key] = value

    def __cmp__(self, other):
        if isinstance(other, Struct):
            return cmp(self.__dict__, other.__dict__)
        else:
            return cmp(self.__dict__, other)

    def __repr__(self):
        args = ['%s=%s' % (k, repr(v)) for (k, v) in vars(self).items()]
        return 'Struct(%s)' % ', '.join(args)

class Enum(object):
    """ 
    Simple enum class.  Usage: 
    >>> FruitEnum = Enum("Banana Apple Cherry".split()"
    >>> print FruitEnum.Cherry
    Cherry
    """
    def __init__(self, *names):
        for name in names:
            self.__dict__[name] = name
        self.names = set(names)
    def values(self):
        return self.names

    
@contextmanager
def log_time(name, log_fn):
    t = time.time()
    yield
    log_fn("%s took %.2f sec", name, time.time()-t)

def update(x, **entries):
    """Update a dict; or an object with slots; according to entries.
    >>> update({'a': 1}, a=10, b=20)
    {'a': 10, 'b': 20}
    >>> update(Struct(a=1), a=10, b=20)
    Struct(a=10, b=20)
    """
    if isinstance(x, dict):
        x.update(entries)   
    else:
        x.__dict__.update(entries) 
    return x 


def is_string(val):
    return isinstance(val, (str,basestring))
        
def is_seq(obj):
    try:
        obj[0:0]
        return True
    except:
        return False

def is_seq_but_not_string(obj):
    return is_seq(obj) and not is_string(obj)

def uniq(iterable):
    """ generates a unique sequence from an iterable but keeps the original
    order."""
    seen = set()
    for elmt in iterable:
        if elmt not in seen:
            seen.add(elmt)
            yield elmt

def multiple_replace(text, adict):
    # from Python Cookbook (2nd ed.), p.38ff
    rx = re.compile('|'.join(map(re.escape, adict)))
    def one_xlat(match):
        return adict[match.group(0)]
    return rx.sub(one_xlat, text)
    
def rec_replace(alist, repl_dict, is_seq=is_seq_but_not_string, delete_if_repl_empty=True):
    # recursively replace elements in nested lists
    # returns all replaced elements
    repl = set()
    for i, elmt in enumerate(alist):
        if is_seq(elmt):
            repl.update(rec_replace(elmt, repl_dict))
        elif elmt in repl_dict:
            r = repl_dict[elmt]
            repl.add(elmt)
            if r != "":
                alist[i] = r
            else:
                del alist[i:i+1]
    return repl

def flatten(seq, is_seq=is_seq_but_not_string):
    iterator = iter(seq)
    for elmt in iterator:
        if is_seq(elmt):
            for sub_elmt in flatten(elmt, is_seq):
                yield sub_elmt
        else:
            yield elmt
    
def strlists2columns(lists, sep=3, width=None):
    if not width:
        width = sep
        for l in lists:
            for r in l:
                width = max(width, len(r)+sep)
    newlists = [[s.ljust(width) for s in l] for l in lists]
    transp = zip(*newlists)
    s = "\n".join(["".join(l) for l in transp])
    return s

def it2str(it):
    # TODO: apply it2str(elmt) too!
    return [str(elmt) for elmt in it]
    
def str2python_obj(val, clean_first=True, comment_sign="#"):
    special_values = {"true":True, "false":False, "none":None}
    if clean_first:
        val = val.split(comment_sign, 1)[0].strip()  # remove comments and whitespace
    if is_string(val) and val.isdigit():
        val = int(val)
    elif val.lower() in special_values:
        val = special_values[val.lower()]
    else:
        try:
            if val[0] == val[-1] and val[0] in ("'", '"'):
                val = val[1:-1]
        except:
            pass
    return val

def read_property_file(fn, comment_sign="#"):
    """ reads in a file of the following structure and
    returns a dict.  If possible turns strings into python
    objects ("true" to True etc) on the fly. file format:
    property_name value
    property_name value
    """
    def prop_gen(fn):
        for line_no, line in enumerate(open(fn)):
            line = line.strip()
            if not line or line[0] == comment_sign:
                continue
            tokens = line.split()
            if len(tokens) == 1:
                tokens.append("")
            elif len(tokens) > 2:
                tokens[1] = " ".join(tokens[1:])
            prop_name, prop_val = tokens[0:2]
            prop_val = str2python_obj(prop_val)
            yield prop_name, prop_val
    return dict((prop_name, prop_val) for prop_name, prop_val in prop_gen(fn))

def select_elements(iterable, predicate):
    """separates the elements of an iterable into two lists, one
       where alle elements satisfy a given predicate, and one where
       they don't. Die guten ins Toepfchen, die schlechten ins Kroepfchen."""
    d = {True: [], False: []}
    for elmt in iterable:
        d[predicate(elmt)].append(elmt)
    return d[True], d[False]

def izip_longest(*args, **kwds):
    ''' Alternate version of izip() that fills-in missing values rather than truncating
    to the length of the shortest iterable.  The fillvalue is specified as a keyword
    argument (defaulting to None if not specified).

    >>> list(izip_longest('a', 'def', 'ghi'))
    [('a', 'd', 'g'), (None, 'e', 'h'), (None, 'f', 'i')]
    >>> list(izip_longest('abc', 'def', 'ghi'))
    [('a', 'd', 'g'), ('b', 'e', 'h'), ('c', 'f', 'i')]
    >>> list(izip_longest('a', 'def', 'gh'))   
    [('a', 'd', 'g'), (None, 'e', 'h'), (None, 'f', None)]
    '''
    fillvalue = kwds.get('fillvalue')
    def sentinel(counter=[fillvalue]*(len(args)-1)):
        yield counter.pop()     # raises IndexError when count hits zero
    iters = [chain(it, sentinel(), repeat(fillvalue)) for it in args]
    try:
        for tup in izip(*iters):
            yield tup
    except IndexError:
        pass

##########################################################
# Memoize function results and attributes:

def memoize(f, cache={}):
    """ 
    Memoize calls to a function for specific args/kwargs arguments.
    Attention: make sure to call with the same named/unnamed params
    all the time, ie don't call g(1) once and g(x=1) later - this would
    lead to a different key!
    """    
    def g(*args, **kwargs):
        key = ( f, tuple(args), frozenset(kwargs.items()) )
        if key not in cache:
            cache[key] = f(*args, **kwargs)
        return cache[key]
    return g


# From the Python Cookbook Ver 2, Recipe 20.4
class cached_attribute(object):
    """
    Computes attribute value and caches it in the instance.
    """
    def __init__(self, method, name=None):
        # record the unbound-method and the name
        self.method = method
        self.name = name or method.__name__
    def __get__(self, inst, cls):
        if inst is None:
            # instance attribute accessed on class, return self
            return self
        # compute, cache and return the instance's attribute value
        result = self.method(inst)
        setattr(inst, self.name, result)
        return result
    
class cached_class_attribute(cached_attribute):
    """
    Computes attribute value and caches it in the class.
    """
    def __get__(self, inst, cls):
        # just delegate to CachedAttribute, with 'cls' as ``instance''
        return super(CachedClassAttribute, self).__get__(cls, cls)



##################################################################

def load_config_file(filename, as_struct=True, base_dict={}):
    from configobj import ConfigObj
    def add_config_items(obj, adict):
        for key, val in adict.items():
            if isinstance(val, dict):
                nval = Struct()
                add_config_items(nval, val)
                val = nval
            elif isinstance(val, str):
                if val.lower() in ("true", "yes"):
                    val = True
                elif val.lower() in ("false", "no"):
                    val = False
                else:
                    try:
                        val = int(val)
                    except:
                        try:
                            val = float(val)
                        except:
                            pass
                
            obj.__dict__[key] = val
    base = ConfigObj(dict(DEFAULT=base_dict))
    config_dict = ConfigObj(filename, file_error=True, interpolation="ConfigParser")
    config_dict.merge(base)
    if not as_struct:
        return config_dict
    else:
        config = Struct()
        add_config_items(config, config_dict)
    return config

def codepos():
    """Returns the filename, function name and current line number in our program."""
    frame = inspect.currentframe().f_back
    filename = frame.f_code.co_filename
    funcname = frame.f_code.co_name
    lineno = frame.f_lineno
    return "%s, %s(), line %d:\n" % (filename, funcname, lineno)

def bit_is_set(bitfield, bitnum):
    return bool(bitfield & bitnum)

def clear_screen():
    os.system("clear")

def wait_for_keypress(prompt=""):
    raw_input(prompt)

def is_dated(cfn, fn):
    from os.path import exists, getmtime
    return not exists(cfn) or getmtime(cfn) < getmtime(fn)

def rmgeneric(path, __func__):
    assert __func__ in [os.remove, os.rmdir]
    try:
        __func__(path)
    except OSError, (errno, strerror):
        print "Warning: Error removing %(path)s, %(error)s" % {'path' : path, 'error': strerror }

def removeall(path):
    if not os.path.isdir(path):
        return
    files=os.listdir(path)
    for x in files:
        fullpath=os.path.join(path, x)
        if os.path.isfile(fullpath):
            f=os.remove
            rmgeneric(fullpath, f)
        elif os.path.isdir(fullpath):
            removeall(fullpath)
            f=os.rmdir
            rmgeneric(fullpath, f)

def run_command(cmd, input=None, output=None):
    """
    Runs a command, possibly with redirected input/output.
    Reads and returns the output if an output file name was given.
    Currently no piped version (due to possible buffering blocks with piped output)
    """
    import subprocess
    redirections = {}
    if input:
        redirections["stdin"] = open(input)
    if output:
        redirections["stdout"] = open(output, "w")
        redirections["stderr"] = open(output, "w")
    subprocess.call(cmd.split(), **redirections)
    if output:
        return open(output).read()
    return None

def run_process(cmd, input=None, output=PIPE, error=PIPE, dir=None, wait=True):
    """
    Runs a command, possibly with redirected input/output.
    Reads and returns the output if an output file name was given.
    Currently no piped version (due to possible buffering blocks with piped output)
    """
    import subprocess
    redirections = {}
    if input:
        if isinstance(input, file):
            redirections["stdin"] = input
        else:
            redirections["stdin"] = open(input)
    if output != PIPE:
        redirections["stdout"] = open(output, "w")
    else:
        redirections["stdout"] = PIPE
    if error not in (PIPE, STDOUT):
        redirections["stderr"] = open(error, "w")
    else:
        redirections["stderr"] = error

    if dir:
        redirections["cwd"] = dir
        
    process = subprocess.Popen(cmd.split(), **redirections)
    if wait:
        out, err = process.communicate()
        return process, out, err
    return process

def print_errors(process, cmd, output, logger, name=None):
    if not name:
        executable = cmd.split()[0]
        name = os.path.basename(executable)

    if process.returncode > 0:
        logger.warning("%s returned with nonzero %d.", name, process.returncode)
        lfunc = logger.warning
    else:
        logger.error("%s was killed by signal %d.", name, -process.returncode)
        lfunc = logger.error
    lfunc("Call was: %s", cmd)
    lfunc("Output:")
    lfunc("=====================================================================")
    lfunc(output)
    lfunc("=====================================================================")

def run_unit_tests(args):
    pass

if __name__ == "__main__":
    import sys
    run_unit_tests(sys.argv[1:])
