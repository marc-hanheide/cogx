#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009 

from mods import comparator

class CObjectMatcher:
    def __init__(self):
        self.bestViews = None 
        self.dists = None
        self.scores = None

    # Lower score is better
    def calcDistScore(self, dists, curBest=1e99):
        if len(dists) < 1: return 1e99
        score = 0
        for i in xrange(20):
            if i >= len(dists): # no more matches
                el = dists[-1]
                score += el[1][0][0] * (2.0 + i / 40.0)
            else:
                el = dists[i]
                score += el[1][0][0] * (1.0 + i / 40.0)
            if score > curBest: return score
        return score

    # model - CObjectModel
    # example - Featurepack
    def match(self, model, example, goodOnly = False):
        comp = comparator.CDescriptorMatcher(nBest = 2)
        bestvx = None; bestScore = 1e99; bestPairs = []
        self.bestViews = None
        self.dists = None
        self.scores = None
        bestviews = comparator.CPriq(maxcount = 5)
        for vpidx, vx in enumerate(model.viewPoints):
            allgrps = vx.featurePacks[0]

            # print vpidx, vx.id
            dists = comp.matchLists(example.descriptors, allgrps.descriptors, maxRatio=0.8)
            if len(dists) < 1: continue

            dists.sort(key=lambda x: x[1][0])
            score = self.calcDistScore(dists)

            bestviews.add([vx, dists], score)

        if len(bestviews.toplist) > 0:
            self.bestViews = [ view[1][0] for view in bestViews.toplist]
            self.dists = [ view[1][1] for view in bestViews.toplist]
            self.scores = [ view[0] for view in bestViews.toplist]
            # TODO: implement voting
            # TODO: approximate pose estimation - rotations are missing 
            #for el in bestviews.toplist:
               #print "Score", el[0], el[1][0].id

        return len(self.bestViews)
