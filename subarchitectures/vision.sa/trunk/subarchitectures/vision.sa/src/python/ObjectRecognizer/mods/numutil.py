#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jan 2009 
import numpy as np
import scipy as sci
import math

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

class CAngleAverage(object):
    def __init__(self, default=None):
        self.sumangle = [0.0, 0.0]
        self.sumweight = [0.0, 0.0]
        self.default = default

    def add(self, angle, weight=1.0):
        angle = math.fmod(angle, 2*math.pi)
        i = 0 if angle < math.pi else 1
        self.sumangle[i] += angle*weight
        self.sumweight[i] += weight

    def addDeg(self, angle, weight=1.0):
        self.add(angle * math.pi / 180)

    @property
    def average(self):
        if self.sumweight[0] == 0 and self.sumweight[1] == 0: return self.default
        if self.sumweight[0] == 0: return self.sumangle[1] / self.sumweight[1]
        if self.sumweight[1] == 0: return self.sumangle[0] / self.sumweight[0]
        a = [self.sumangle[0] / self.sumweight[0], self.sumangle[1] / self.sumweight[1] ]
        if (2*math.pi - a[1]) + a[0] <= math.pi:
            self.sumangle[1] = -(2*math.pi - a[1]) * self.sumweight[1]
            delta = 0
        else:
            self.sumangle[0] = - (math.pi - a[0]) * self.sumweight[0]
            self.sumangle[1] = (a[1] - math.pi) * self.sumweight[1]
            delta = math.pi
        ave = (self.sumangle[0] + self.sumangle[1]) / (self.sumweight[0] + self.sumweight[1]) + delta
        ave = math.fmod(ave, 2*math.pi)
        if ave < 0: ave += 2*math.pi
        return ave

    @property
    def averageDeg(self):
        a = self.average
        if a == None: return a
        else: return a * 180 / math.pi


if __name__=="__main__":
    def testAngleAverage(angles, weights, result, eps=1e-3):
        a = CAngleAverage()
        for i in xrange(len(angles)):
            ang = angles[i] * math.pi / 180
            if weights == None or i >= len(weights): a.add(ang)
            else: a.add(ang, weights[i])
        rdeg = math.fmod(a.average * 180 / math.pi, 360)
        res = abs(rdeg - result) < eps
        return res, rdeg

    angles = [10, 20, 30]; weights = None
    print angles, testAngleAverage(angles, weights, 20)
    angles = [210, 220, 230]; weights = None
    print angles, testAngleAverage(angles, weights, 220)
    angles = [170, 180, 190]; weights = None
    print angles, testAngleAverage(angles, weights, 180)
    angles = [350, 0, 10]; weights = None
    print angles, testAngleAverage(angles, weights, 0)
    angles = [10, 20, 30, 210, 220, 230]; weights = None
    print angles, testAngleAverage(angles, weights, 300)
    angles = [350, 0, 10]; weights = [3, 1, 3]
    print angles, testAngleAverage(angles, weights, 0)
    angles = [160, 180, 190]; weights = [2, 1, 1]
    print angles, testAngleAverage(angles, weights, 172.5)
    angles = [340, 0, 10]; weights = [2, 1, 1]
    print angles, testAngleAverage(angles, weights, 352.5)

