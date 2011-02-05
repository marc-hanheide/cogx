#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009 

import math
import numpy as np
from scipy.cluster import hierarchy
from mods import comparator, numutil
from osmods import greatcircle # distances on ellipsoids

import logging
LOG = logging.getLogger("ObjectRecognizer")

class CConsistency(object):
    def __init__(self, dists, fpackExample, fpackModel, nOrientations, nScales):
        self.dists = dists
        self.fpExample = fpackExample
        self.fpModel = fpackModel
        self.nOrientations = nOrientations
        self.nScales = nScales
        self.rotation = 0
        self.rotationConfidence = 0
        self.rotationDistinctiveness = 0
        self.scale = 0
        self.scaleConfidence = 0
        self.scaleDistinctiveness = 0

    @property
    def rotationDeg(self):
        return self.rotation * 180 / math.pi

    def getConsistentFeatures(self, maxRotDeg=15, maxScaleFac=2):
        fpView = self.fpModel
        fpExample = self.fpExample
        iScale = 2; iOrient = 3
        maxrot = math.pi / 180 * maxRotDeg
        pairs = []
        for el in self.dists:
            ie = el[0]; keyExmpl = fpExample.keypoints[ie]
            im = el[1][0][1]; keyView = fpView.keypoints[im]
            # rotations
            rot = math.fmod((keyView[iOrient] - keyExmpl[iOrient]), 2*math.pi)
            drot = math.fmod((self.rotation - rot), 2*math.pi)
            if drot > math.pi: drot = 2*math.pi - drot
            # scales
            s = keyExmpl[iScale] / keyView[iScale]
            if s < 1: s = 1/s
            # consistent:
            # if dor < maxrot and s < maxScaleFac: pairs.append(el)
            if abs(drot) < maxrot: pairs.append(el)
        return pairs

    def dump(self):
        print "Rotation: %.1f Conf: %.2f Dist: %.2f" % (
            self.rotation*180/math.pi, self.rotationConfidence, self.rotationDistinctiveness),
        print "Scale: %.1f Conf: %.2f Dist: %.2f" % (
            self.scale, self.scaleConfidence, self.scaleDistinctiveness)

class CConsistencyCalculator(object):

    def __init__(self, nOrientations = 36, nScales = 36):
        self.nOrientations = nOrientations
        self.nScales = nScales

    def runningMax(self, aList, nAve=3, cyclic=False):
        d = nAve if cyclic else 0
        n = len(aList)
        s=sum(aList[:nAve])
        smax = s; imax = 0
        for i in xrange(nAve, n+d):
            s = s - aList[i-nAve] + aList[i % n]
            if s > smax: smax = s; imax = i-nAve+1
        return (imax, smax)

    def rangeComplement(self, n, iStart, iEnd):
        iStart = iStart % n
        iEnd = iEnd % (n + 1)
        ranges = []
        if iStart < iEnd:
            if iStart > 0: ranges.append( (0, iStart) )
            if iEnd < n: ranges.append( (iEnd, n) )
        else: ranges.append( (iEnd, iStart) )
        return ranges

    def sumOther(self, aList, iStart, iEnd):
        rs = self.rangeComplement(len(aList), iStart, iEnd)
        s = 0
        for r in rs: s += sum(aList[r[0]:r[1]])
        return s

    def maxOther(self, aList, iStart, iEnd):
        rs = self.rangeComplement(len(aList), iStart, iEnd)
        m = -1e99
        for r in rs: m = max(m, max(aList[r[0]:r[1]]))
        return m

    # Find scale and rotation consistency of features
    def getConsistency(self, dists, fpExample, fpView):
        nor = self.nOrientations; nscl = self.nScales
        R = CConsistency(dists, fpExample, fpView, nor, nscl)
        histOrients = [0] * nor
        histScales = [0] * nscl
        iScale = 2; iOrient = 3
        for el in dists:
            ie = el[0]
            im = el[1][0][1]
            # orientations
            rot = math.fmod((fpView.keypoints[im][iOrient] - fpExample.keypoints[ie][iOrient]), 2*math.pi)
            ih = int(rot * nor / (2*math.pi)) % nor
            histOrients[ih] += 1
            # scales
            s = fpExample.keypoints[ie][iScale]/ fpView.keypoints[im][iScale]
            if s < 1: s = - 1/s
            s = int(s*2 + nscl/2 + 0.5)
            if s < 0: s = 0
            if s >= nscl: s= nscl-1
            histScales[s] += 1

        imax, smax = self.runningMax(histOrients, 3, True)
        icnt = imax + sum( (histOrients[(imax+i)%nor] * (i+0.5) for i in xrange(3)) ) / smax
        R.rotation = math.fmod(2*math.pi * icnt / nor, 2*math.pi)
        conf =  1.0 * smax / sum(histOrients)
        R.rotationConfidence = conf
        other = self.maxOther(histOrients, imax-1, imax + 4)
        dist = 1.0 * (smax - other * 3 * (1-conf)) / smax
        R.rotationDistinctiveness = dist

        imax, smax = self.runningMax(histScales, 3, False)
        icnt = imax + sum( (histScales[(imax+i)%nscl] * (i+0.5) for i in xrange(3)) ) / smax
        s = (icnt - nscl/2) / 2.0
        if s < 0: s = -1.0/s
        R.scale = s
        conf =  1.0 * smax / sum(histScales)
        R.scaleConfidence = conf
        other = self.maxOther(histScales, imax-1, imax + 4)
        dist = 1.0 * (smax - other * 3 * (1-conf)) / smax
        R.scaleDistinctiveness = dist

        return R

class CObjectMatcher:
    def __init__(self):
        self.bestViews = []
        self.consists = []
        self.scores = []
        self.totalScore = 0.0
        self.descriptorMatcher = None

    # Calculate a score from an (ordered) array of distnaces.
    # (Higher score is better. Views with more keypoints will score better.)
    # TODO: d0 could be a model/view parameter
    def calcDistScore(self, dists):
        if len(dists) < 1: return 0
        d0 = 1e-3
        score = 0
        for i,el in enumerate(dists):
            d = el[1][0][0]
            if d < d0: d=d0
            score += 1.0/d
        return score

    def getTotalScore(self):
        if len(self.scores) == 0: return 1e99
        return self.scores[0]

    def _clear(self):
        self.bestViews = []
        self.consists = []
        self.scores = []
        self.totalScore = 0.0

    def _setBestResults(self, bestViewList):
        top = bestViewList.getTopTuples()
        if len(top) < 1:
            self._clear()
            return
        scmin = top[0][0] * 0.55
        scmin = 0
        gv = [atuple for atuple in top if atuple[0] > scmin]
        self.bestViews = [ view[1][0] for view in gv]
        self.consists = [ view[1][1] for view in gv]
        self.scores = [ view[0] for view in gv]
        self.totalScore = bestViewList.totalSum

    #def quickMatch(self, model, example):
    #    if model == None or example == None: return 0
    #    self._clear()
    #    comp = comparator.CDescriptorMatcher(nBest = 2)
    #    levels, phis = model.arrangeViewsByPhi()
    #    lev = levels[0]
    #    ll = len(lev)
    #    vpts = [ lev[0], lev[int(ll * .25)], lev[int(ll * .5)], lev[int(ll * .75)] ]
    #    bvsort = comparator.CPriq(maxcount = 5, best=max)
    #    for vpidx, vx in enumerate(vpts):
    #        allgrps = vx.featurePacks[0]
    #
    #        # print vpidx, vx.id
    #        dists = comp.matchLists(example.descriptors, allgrps.descriptors, maxRatio=0.8)
    #        if len(dists) < 1: continue
    #
    #        dists.sort(key=lambda x: x[1][0])
    #        score = self.calcDistScore(dists)
    #
    #        bvsort.add([vx, dists], score)
    #
    #    self._setBestResults(bvsort)
    #    return len(self.bestViews)

    # model - CObjectModel
    # example - Featurepack
    def match(self, model, example):
        if model == None or example == None: return 0
        comp = self.descriptorMatcher
        self._clear()
        bvsort = comparator.CPriq(maxcount = len(model.viewPoints)/2, best=max)
        consistCalc = CConsistencyCalculator()
        for vpidx, vx in enumerate(model.viewPoints):
            allgrps = vx.featurePacks[0]

            res = comp.homographyMatchLists(example.descriptors, allgrps.descriptors, maxRatio=0.8)
            (dists, hmgrMatch, homography) = res
            if len(dists) < 1: continue
            if hmgrMatch == None: numHmgrMatch = 0
            else: numHmgrMatch = len(hmgrMatch)

            LOG.debug("Descriptors: (Ex=%d, M=%d), Matched: sifts=%d, with hmgr=%d" % (
                len(example.descriptors), len(allgrps.descriptors), len(dists), numHmgrMatch
            ))

            # print "BEFORE ", dists[:10]
            # dists.sort(key=lambda x: x[1][0])
            # print "AFTER ", dists[:10]

            maxMatches = len(vx.featurePacks[0])
            # H: shorten if the example has more features than the view; 
            #    duplicate matches may still remain among best scores
            dists = dists[:maxMatches]
            score = self.calcDistScore(dists)
            consist = consistCalc.getConsistency(dists, example, allgrps)
            hmgrScore = self.calcDistScore(hmgrMatch)

            cf = consist.getConsistentFeatures(maxRotDeg=20)
            score2 = self.calcDistScore(cf)
            LOG.debug("-- Score: %.1f Consistent: %.2f, Cons.Score: %.1f, ScoreFac: %.2f; HmgrScore: %.2f" % (
               score, 1.0 * len(cf) / len(dists),
               score2, score2/score, hmgrScore)
            )
            #consist.dump()

            bvsort.add([vx, consist], score * 0.1 + hmgrScore)

        self._setBestResults(bvsort)
        # TODO: approximate pose estimation - rotations are missing 
        #for el in bvsort.toplist:
        #    print "Score", el[0], el[1][0].id

        return len(self.bestViews)

    # Probability distribution over best views
    # returns: list of (i, p(i))
    def getProbabilityDistribution(self, goodScoreRatio=0.5):
        probGood = 0.9 # The good-matching views represent 90% of total probability
        scmin = self.scores[0] * goodScoreRatio
        scores = [sc for sc in self.scores if sc > scmin]
        stotal = sum(scores)
        # print "Good-matching 'energy':", stotal / self.totalScore
        probs = [p / stotal for p in scores]
        probs = [p * probGood for p in probs]
        return probs


    # Cluster best views and return a point from each cluster
    # returns [ (P, phi, lambda, rot) ] ; maybe todo: support, confidence
    def getPoseProbability(self):
        N = len(self.bestViews)
        if N < 1: return []
        if N == 1:
            bv = self.bestViews[0]
            return [ (1.0, bv.vpPhi, bv.vpLambda, self.consists[0].rotation) ]

        def _viewDistance(v1, v2):
            def rad(x): return x * math.pi / 180
            r = 1.0
            d,a1,a2 = greatcircle.vinc_dist(0.0, r, rad(v1.vpPhi), rad(v1.vpLambda), rad(v2.vpPhi), rad(v2.vpLambda))
            circ = 2 * math.pi * r
            if d > circ / 2: d = circ - d
            return d

        if N == 2: pass # TODO
        dm = np.ndarray(shape=(N,N), dtype = np.float)
        for i in xrange(N):
            for j in xrange(N):
                if i==j: dm[i,j] = 0
                else: dm[i,j] = _viewDistance(self.bestViews[i], self.bestViews[j])

        hcLinks = hierarchy.complete(dm)
        # clusterIndex = hierarchy.fcluster(hcLinks, t=0.8)
        nc = N/2
        if nc < 4: nc = 4
        if nc < N: nc = N
        if nc > 8: nc = 8
        clusterIndex = hierarchy.fcluster(hcLinks, t=nc, criterion='maxclust')
        clusters = set(clusterIndex)
        clpoints = [] # (score, phi, lambda, rot)
        for cn in clusters:
            score = 0; nsc = 0
            aphi = numutil.CAngleAverage()
            alam = numutil.CAngleAverage()
            arot = numutil.CAngleAverage()
            for i in xrange(len(clusterIndex)):
                if clusterIndex[i] != cn: continue
                bv = self.bestViews[i]
                aphi.addDeg(bv.vpPhi, self.scores[i])
                alam.addDeg(bv.vpLambda, self.scores[i])
                score += self.scores[i]; nsc += 1
                conf = self.consists[i]
                if conf.rotationConfidence > 0.4:
                    arot.add(conf.rotation, self.scores[i])
            if nsc>0:
                bonus = (1.0 + 1.0 * nsc / N) # H: for cluster size
                rot = arot.averageDeg
                if rot == None: rot = self.consists[0].rotation * 180 / math.pi
                clpoints.append( (bonus*score/nsc, aphi.averageDeg, alam.averageDeg, rot) )

        # H: Normalize scores -> probabilities
        score = 0
        for clp in clpoints: score += clp[0]
        clpoints = [ (clp[0] / score, clp[1], clp[2], clp[3]) for clp in clpoints ]
        return sorted(clpoints)

# An attempt to upgrade the recognizer to a detector
# TODO: Use geometric matching to score each view
#class CObjectDetector(CObjectMatcher):
#    def __init__(self):
#        self.bestViews = []
#        self.consists = []
#        self.scores = []
#        self.totalScore = 0.0
#        self.descriptorMatcher = None

#    # model - CObjectModel
#    # example - Featurepack
#    def match(self, model, example):
#        if model == None or example == None: return 0
#        comp = self.descriptorMatcher
#        self._clear()
#        bvsort = comparator.CPriq(maxcount = len(model.viewPoints)/2, best=max)
#        consistCalc = CConsistencyCalculator()
#        for vpidx, vx in enumerate(model.viewPoints):
#            allgrps = vx.featurePacks[0]

#            res = comp.homographyMatchLists(example.descriptors, allgrps.descriptors, maxRatio=0.8)
#            (dists, hmgrMatch, homography) = res

#            if len(dists) < 1: continue
#            if hmgrMatch == None: numHmgrMatch = 0
#            else: numHmgrMatch = len(hmgrMatch)
#            LOG.debug("Descriptors: (%d, %d), Matched sifts %d, Matched with hmgr: %d" % (
#                len(example.descriptors), len(allgrps.descriptors), len(dists), numHmgrMatch
#            ))

#            dists.sort(key=lambda x: x[1][0])
#            maxMatches = len(vx.featurePacks[0])
#            # H: shorten if the example has more features than the view; 
#            #    duplicate matches may still remain among best scores
#            scfactor = 1.0
#            if numHmgrMatch < 4: scfactor = numHmgrMatch / 4.0
#            dists = dists[:maxMatches]
#            score = self.calcDistScore(dists) * scfactor
#            consist = consistCalc.getConsistency(dists, example, allgrps)

#            bvsort.add([vx, consist], score)

#        self._setBestResults(bvsort)
#        # TODO: approximate pose estimation - rotations are missing 
#        #for el in bvsort.toplist:
#        #    print "Score", el[0], el[1][0].id

#        return len(self.bestViews)

