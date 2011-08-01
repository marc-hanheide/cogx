#!/usr/bin/env python
import readline
import pdb
import socket
import sys
import pprint as pp

skt = None
clientsocket = None
handle = None

class ReadablePP(pp.PrettyPrinter):
    def format(self, object, context, maxlevels, level):
        """Format object for a specific context, returning a string
        and flags indicating whether the representation is 'readable'
        and whether the object represents a recursive construct.
        """
        if not isinstance(object, (dict, list, tuple, set, str)):
            try:
                rep = str(object)
            except:
                rep = repr(object)
            return rep, (rep and not rep.startswith('<')), False
        return pp.PrettyPrinter.format(self, object, context, maxlevels, level)

def pprint(object, stream=None, indent=1, width=80, depth=None):
    """Pretty-print a Python object to a stream [default is sys.stdout]."""
    printer = ReadablePP(
        stream=stream, indent=indent, width=width, depth=depth)
    printer.pprint(object)
    

class Rdb(pdb.Pdb):
    def __init__(self, port=4444):
        global skt, handle, clientsocket
        self.old_stdout = sys.stdout
        self.old_stdin = sys.stdin
        if clientsocket is None:
            if skt is None:
                skt = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                skt.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                skt.bind(('localhost', port))
            skt.listen(1)
            (clientsocket, address) = skt.accept()
            handle = clientsocket.makefile('rw')
        pdb.Pdb.__init__(self, completekey='tab', stdin=handle, stdout=handle)
        sys.stdout = sys.stdin = handle

    def user_line(self, frame):
        if handle is None:
            self.do_continue(None)
            return
        sys.stdout = sys.stdin = handle
        pdb.Pdb.user_line(self, frame)

    def do_continue(self, arg):
        sys.stdout = self.old_stdout
        sys.stdin = self.old_stdin
        self.set_continue()
        return 1

    def do_exit(self, arg):
        global skt, handle, clientsocket
        self.clear_all_breaks()
        sys.stdout = self.old_stdout
        sys.stdin = self.old_stdin
        clientsocket.shutdown(socket.SHUT_RDWR)
        clientsocket = None
        handle = None
        skt.shutdown(socket.SHUT_RDWR)
        skt.close()
        skt = None
        self.set_continue()
        return 1

    def do_pp(self, arg):
        try:
            pprint(self._getval(arg), self.stdout)
        except:
            pass
    
    do_c = do_cont = do_continue
    do_e = do_exit

def set_trace():
    print "Entering debugger, please telnet to localhost:4444"
    Rdb().set_trace()

# Post-Mortem interface

def post_mortem(t=None):
    # handling the default
    if t is None:
        # sys.exc_info() returns (type, value, traceback) if an exception is
        # being handled, otherwise it returns None
        t = sys.exc_info()[2]
        if t is None:
            raise ValueError("A valid traceback must be passed if no "
                                               "exception is being handled")

    p = Rdb()
    p.reset()
    p.interaction(None, t)

def pm():
    post_mortem(sys.last_traceback)
  
