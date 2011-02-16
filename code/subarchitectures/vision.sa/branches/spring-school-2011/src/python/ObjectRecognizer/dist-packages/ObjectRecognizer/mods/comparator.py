#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: may 2009 
import os, sys
import numpy as np
import scipy as sci
import heapq

"""
Calculate distances between descriptors that are represented with vectors.
"""

# A distance calculator class. Default distance is euclidean.
class CComparator(object):
    def __init__(self):
        pass

    # euclidean distance
    def distance(self, f1, f2):
        c = f2 - f1
        return np.sqrt(np.dot(c, c))
 
    # all euclidean distances between elements of two lists
    # list1, list2: 
    #    1. python list of 1D ndarrays
    #    2. rows of a 2D ndarray
    # TODO: do I need maxdist, maxcount, sort?
    def distanceAll(self, list1, list2):
        result = []
        for i1 in xrange(len(list1)):
            for i2 in xrange(len(list2)):
                c = list2[i2] - list1[i1]
                d = np.sqrt(np.dot(c,c))
                result.append( (d, i1, i2) )
        return result

    # all euclidean distances from feature to elements of a list
    def distancesL(self, f, list1):
        result = []
        for i1 in xrange(len(list1)):
            c = list1[i1] - f
            d = np.sqrt(np.dot(c,c))
            result.append( (d, i1) )
        return result

    # all euclidean distances from feature to rows of an array
    def distancesA(self, f, array1):
        diff = array1 - f # subtract f from every row
        sqdiff = diff*diff
        # sum/reduce each row
        diff = sqdiff.sum(axis=1)
        return np.sqrt(diff)

class CPriq(object):
    def __init__(self, maxcount=2, best=min):
        self.toplist = [] # list of tuples; one of tuple elements is the priority
        self.totalSum = 0 # sum of all added priorities
        self.pushOp = heapq.heappush
        self.order = 0 if best == min else 1
        if maxcount < 1: maxcount = 1
        self.maxcount = maxcount

    def clear(self):
        self.toplist = []
        self.totalSum = 0
        self.pushOp = heapq.heappush

    def _sortInsertTuple(self, elem, idxpri = 0):
        l = len(self.toplist)
        if self.order == 0:
            while l > 0 and elem[idxpri] < self.toplist[l-1][idxpri]: l -= 1
        else:
            while l > 0 and elem[idxpri] > self.toplist[l-1][idxpri]: l -= 1
        self.toplist.insert(l, elem)

    # @param elem: tuple to be added
    # @param idxpri: index of tuple element with priority info
    def _addTuple_slow(self, elem, idxpri = 0):
        self.totalSum += elem[idxpri]
        l = len(self.toplist)
        if l < self.maxcount: self._sortInsertTuple(elem, idxpri)
        else:
            last = l - 1
            if self.order == 0: doadd = elem[idxpri] < self.toplist[last][idxpri]
            else: doadd = elem[idxpri] > self.toplist[last][idxpri]
            if doadd:
                self.toplist = self.toplist[:last]
                self._sortInsertTuple(elem, idxpri)

    # implementation with the heap
    def getTopItems(self):
        elms = [ heapq.heappop(self.toplist)[1] for i in xrange(len(self.toplist)) ]
        elms.reverse()
        return elms

    def getTopTuples(self):
        elms = [ heapq.heappop(self.toplist) for i in xrange(len(self.toplist)) ]
        if self.order == 0: elms = [ (-e[0], e[1]) for e in elms]
        elms.reverse()
        return elms

    def _addTuple(self, elem):
        self.totalSum += elem[0]
        if self.pushOp == heapq.heappush and len(self.toplist) >= self.maxcount:
            self.pushOp = lambda l,e: (heapq.heappush(l,e), heapq.heappop(l)) # 2.6 -> heapq.heappushpop
        self.pushOp(self.toplist, elem)
        pass

    def add(self, elem, priority):
        if self.order == 0: self._addTuple( (-priority, elem))
        else: self._addTuple( (priority, elem))


class CDescriptorMatcher(object):
    def __init__(self, nBest=2, comparator=None, best=min):
        self.compare = CComparator() if comparator == None else comparator
        self.nBest = nBest
        self.order = 0 if best == min else 1

    # Match feature f to all features in list1 and sort by distance
    def matchL(self, f, list1):
        tdists = self.compare.distancesL(f, list1)
        tdists.sort()
        if self.nBest < 1: return tdists
        if self.order == 0: return tdists[:self.nBest]
        else:
            sub = tdists[-self.nBest:]; sub.reverse()
            return sub

    # Match feature f to all features (rows) in array1 and sort by distance
    def matchA(self, f, array1):
        dists = self.compare.distancesA(f, array1)
        iidx = dists.argsort()
        if self.nBest < 1: return [(dists[i], i) for i in iidx]
        if self.order == 0:
            return [(dists[i], i) for i in iidx[:self.nBest]]
        else:
            subidx = iidx[-self.nBest:]; subidx.reverse()
            return [(dists[i], i) for i in subidx]

    # Match features from 2 lists, sort by distance and retain the ones with
    # an acceptable best-to-second-best ratio (1 - all are acceptable, 0 - nothing is acceptable)
    # Returns: List of lists:
    #     [ (i, [(d, j), ...]), ...]
    #     i - index of feature from list1
    #     j - index of feature from list2
    #     d - distance between two features
    def matchLists(self, list1, list2, maxRatio=1.0):
        if maxRatio > 1.0: maxRatio = 1.0
        if maxRatio <= 0.0: return []
        if isinstance(list2, np.ndarray) and (isinstance(list1, np.ndarray) or isinstance(list1[0], np.ndarray)):
            fnMatch = self.matchA
        else: fnMatch = self.matchL
        dists = []
        for i, f in enumerate(list1):
            cool = fnMatch(f, list2)
            if maxRatio < 1.0 and len(cool) > 1:
                if cool[1][0] > 0 and cool[0][0] / cool[1][0] > maxRatio: cool = None
            if cool != None and len(cool) > 0:
                dists.append( (i, cool) )
        return dists


def test_cpriq():
    def _doit(tests, elems, check):
        for T in tests:
            print "Queue size:", T.maxcount
            for e in elems: T.add(e, e); print e,
            print
            print "Best:"
            for i, e in enumerate(T.getTopItems()):
                print "   ", e,
                if e == check[i]: print "ok"
                else: print "error"

    sizes = [1, 2, 5]
    elems = [ 8, 4, 2, 7, 1, 9, 2, 3, 11, 4, 17 ]
    tests = [ CPriq(i) for i in sizes ]
    check = [ i for i in [1, 2, 2, 3, 4]]
    print "*** CPriq: Lowest"
    _doit(tests, elems, check)
    tests = [ CPriq(i, best=max) for i in sizes ]
    check = [ i for i in [17, 11, 9, 8, 7]]
    print "*** CPriq: Highest"
    _doit(tests, elems, check)
    pass

if __name__ == "__main__": test_cpriq()

