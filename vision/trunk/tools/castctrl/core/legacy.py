#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim

import collections, itertools, heapq
import sys

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
