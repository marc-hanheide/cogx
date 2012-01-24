#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim

import collections, itertools, heapq
import sys
import os, os.path

pyver = sys.version_info[0] * 100 + sys.version_info[1]

def deque(iterable=[], maxlen=None):
    if maxlen == None or maxlen == 0: return collections.deque(iterable)
    if pyver >= 206: return collections.deque(iterable, maxlen)
    return collections.deque(iterable)

def getMergeFn():
    if pyver >= 206: return heapq.merge

    def xmerge3(*ln):
        heap = []
        for i in itertools.chain(*ln): heap.append(i)
        heapq.heapify(heap)
        while len(heap): yield heapq.heappop(heap)

    return xmerge3

# available since 2.6
def os_path_relpath(target, base=os.curdir):
    if pyver >= 206: return os.path.relpath(target, base)
    
    # http://code.activestate.com/recipes/302594/
    if not os.path.exists(target):
        raise OSError, 'Target does not exist: '+target

    if not os.path.isdir(base):
        raise OSError, 'Base is not a directory or does not exist: '+base

    base_list = (os.path.abspath(base)).split(os.sep)
    target_list = (os.path.abspath(target)).split(os.sep)

    # On the windows platform the target may be on a completely different drive from the base.
    if os.name in ['nt','dos','os2'] and base_list[0] <> target_list[0]:
        raise OSError, 'Target is on a different drive to base. Target: '+target_list[0].upper()+', base: '+base_list[0].upper()

    # Starting from the filepath root, work out how much of the filepath is
    # shared by base and target.
    for i in range(min(len(base_list), len(target_list))):
        if base_list[i] <> target_list[i]: break
    else:
        # If we broke out of the loop, i is pointing to the first differing path elements.
        # If we didn't break out of the loop, i is pointing to identical path elements.
        # Increment i so that in all cases it points to the first differing path elements.
        i+=1

    rel_list = [os.pardir] * (len(base_list)-i) + target_list[i:]
    return os.path.join(*rel_list)


