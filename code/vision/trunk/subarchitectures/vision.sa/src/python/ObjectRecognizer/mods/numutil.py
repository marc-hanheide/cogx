#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jan 2009 
import numpy as np
import scipy as sci

# a, b - 2 arrays (of sift descriptors) being matched
# maxdist - maximum alowed distance
def mindistances(a, b, maxcount=0, maxdist=0.5):
    result = []
    for ia in xrange(len(a)):
        for ib in xrange(len(b)):
            c = a[ia]-b[ib]
            f = np.sqrt(np.dot(c,c)) # euclidean distance
            if maxdist <= 0 or f <= maxdist: result.append((f, ia, ib))
    # Sorting is VERY slow (all distances=0.01s, sort=0.1s); 
    # Maybe numpy.argsort() could help if the results were a ndarray
    result.sort(key = lambda x: x[0])
    if maxcount < 1: return result
    return result[:maxcount]

def siftdistance(descrA, descrB):
    c = descrB - descrA
    f = np.sqrt(np.dot(c,c)) # euclidean distance
    return f

def siftdistancew(descrA, descrB, weight):
    # Check Mahalanobis (suggested by AŠ)
    c = (descrB - descrA) * weight # element by element
    f = np.sqrt(np.dot(c,c)) # euclidean distance
    return f

def maxcovar(a, b, maxcount=40, maxdist=0.5):
    result = []
    for ia in xrange(len(a)):
        for ib in xrange(len(b)):
            c = sci.correlate(a[ia], b[ib])
            f=c[0]
            result.append((f, ia, ib))
    result.sort(key = lambda x: x[0], reverse = True)
    return result[:maxcount]


def descrsplit(a):
    out = a[0:32].tolist()
    r = [(56,64), (88,96), (120,128), (112,120), (104,112), (96,104), (64,72), (32,40)]
    for (i, j) in r: out.extend(a[i:j])
    ein = a[40:56].tolist()
    ein.extend(a[80:88])
    ein.extend(a[72:80])
    return (out, ein)

def maxcovar2(a, b, maxcount=40, maxdist=0.5):
    result = []
    for ia in xrange(len(a)):
        for ib in xrange(len(b)):
            aout, ain = descrsplit(a[ia])
            bout, bin = descrsplit(b[ib])
            c1 = sci.correlate(aout, bout)
            c2 = sci.correlate(ain, bin)
            f = c1[0] + c2[0]
            result.append((f, ia, ib))
    result.sort(key = lambda x: x[0], reverse = True)
    return result[:maxcount]

