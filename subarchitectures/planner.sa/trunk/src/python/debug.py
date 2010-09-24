#!/usr/bin/env python
import readline
import pdb
import socket
import sys

class Rdb(pdb.Pdb):
  def __init__(self, port=4444):
    self.old_stdout = sys.stdout
    self.old_stdin = sys.stdin
    self.skt = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.skt.bind(('localhost', port))
    self.skt.listen(1)
    (self.clientsocket, address) = self.skt.accept()
    handle = self.clientsocket.makefile('rw')
    pdb.Pdb.__init__(self, completekey='tab', stdin=handle, stdout=handle)
    sys.stdout = sys.stdin = handle

  def do_continue(self, arg):
    sys.stdout = self.old_stdout
    sys.stdin = self.old_stdin
    self.clientsocket.shutdown(socket.SHUT_RDWR)
    self.skt.shutdown(socket.SHUT_RDWR)
    self.skt.close()
    self.set_continue()
    return 1

  do_c = do_cont = do_continue

def set_trace():
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
  
