#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009 

import os, sys, traceback, time
import mods.cameraview as camview
import mods.comparator as comparator
import mods.timing
import objectmodel
import objectmatcher
import numpy as np

from featuresetup import CSiftSetup
# Because of OpenGL, siftgpu should only be used in one thread.

T = mods.timing.TimerOn()


class CModelManager:
    def __init__(self):
        self.models = {} # (filename, model)

    @property
    def modelNames(self):
        return sorted([model for model in self.models.iterkeys()])

    # Model filenames:
    #   NAME.model.dat.gz: a file with the optimized model
    #   NAME.view.XXXX.dat.gz: a file with data for one view
    def addModel(self, name, directory):
        #filename = "%s/%s.model.dat.gz" % (directory, name)
        #if not os.path.exists(filename):
        #    raise IOError("File does not exist: '%s'" % filename)
        self.models[name] = [None, name, directory]
        self._loadModel(self.models[name])
        pass

    def _loadModel(self, model):
        name, mdldir = model[1], model[2]
        m = objectmodel.CObjectModel(name)
        m.FM.setModelStorePath(mdldir)
        model[0] = m
        print "Loading views"
        m.loadViews(delayedLoad=False)
        print len(m.viewPoints), "views"
        print "Loading matches"
        m.loadPairMatches(mdldir, name)
        print "Loadeth!"

    def getModel(self, name):
        if not self.models.has_key(name): return None
        model = self.models[name]
        if model[0] == None: self._loadModel(model)
        return model[0]

def printMatches(matcher):
    print "Score\tPhi\tLambda\tRot\tRotConf\tScale\tSclConf\tScore2\tScorPct\tNFPct"
    for i,bv in enumerate(matcher.bestViews):
        sc = matcher.scores[i]
        cons = matcher.consists[i]
        print "%.1f\t%.1f\t%.1f\t" % (sc, bv.vpPhi, bv.vpLambda),
        print "%.1f\t%.2f\t%.1f\t%.2f\t" % (
            cons.rotationDeg, cons.rotationConfidence, cons.scale, cons.scaleConfidence),
        cf = cons.getConsistentFeatures(maxRotDeg=20)
        score2 = matcher.calcDistScore(cf)
        print "%.1f\t%.2f\t%.2f" % (score2, score2 / sc, 1.0 * len(cf) / len(cons.dists))
        if i > 15: print "..."; break
        #if sc > sclimit[1]: break
        #prob = 1.0 - (sc - sclimit[0]) / sclimit[1]
        #if prob > 1: prob = 1.0
        #if prob < 0: prob = 0.0
        ## TODO: estimate affine transformation -> roll
        #matches.append( (model.name, float(bv.vpLambda), float(bv.vpPhi), float(0), prob) )
    # print matches
    print "Clusters:"
    print " Prob\tPhi\tLambda\tRot"
    pose = matcher.getPoseProbability()
    pose.reverse()
    for p in pose: print " %.3f\t%.1f\t%.1f\t%.1f" % p

Setup = CSiftSetup(extractor=CSiftSetup.GPU, matcher=CSiftSetup.CUDA)
Matcher = objectmatcher.CObjectMatcher()
Matcher.descriptorMatcher = Setup.matcher
Manager = CModelManager()

# Returns a list of tuples (model_name, lambda(yaw), phi(pitch), roll, probability)
def _findMatchingObject(image):
    global Setup, Matcher, Manager
    print "Image shape", image.shape
    if len(Manager.modelNames) < 1:
        print "No model loaded"
        return []
    exmpl = Setup.extractor.extractFeatures(image)
    matches = []
    sclimit = (4.0, 20.0)
    for mn in Manager.modelNames:
        model = Manager.getModel(mn)
        T.tic(); print "\nMATCH Model %s" % mn
        Matcher.match(model, exmpl) # Matcher.quickMatch(model, exmpl)
        T.toc("Time to match:")
        if len(Matcher.scores) < 1: continue
        pose = matcher.getPoseProbability()
        pose.reverse()
        matches.append( (mn, Matcher.bestViews[:3], Matcher.scores[:3], Matcher.consists[:3], pose) )
        # printMatches(Matcher)
    probs = []
    sp = 0
    for m in matches:
        # H: expected best score; 0.3 matches; d0=0.001, max 10% better; decrease with distance
        dfac = m[3][0].scale * m[3][0].scaleConfidence
        if dfac > 1: dfac = 1
        if dfac <= 1e-3: dfac = 1e-3
        estsc = len(m[1][0].featurePacks[0]) * 0.3 * 100 * dfac
        realsc = m[2][0]
        probs.append( [m[0], realsc / estsc, m[4]] ) # name, %, pose
        sp += realsc / estsc

    if sp > 0.9: # H
        for p in probs: p[1] = 0.9 * p[1] / sp

    return probs

# image: ndarray (h, w, 1 or 3)
# region: (x0, y0, x1, y1)
def findMatchingObject(image, region=None):
    try:
        if region != None:
            image = np.copy(image[x0:x1, y0:y1])
        matches = _findMatchingObject(image)
        return matches
    except:
        exceptionType, exceptionValue, exceptionTraceback = sys.exc_info()
        traceback.print_exception(exceptionType, exceptionValue, exceptionTraceback)
        # traceback.print_tb(exceptionTraceback)
    return None
