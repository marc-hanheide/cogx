#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009 

import os, sys, time
import mods.cameraview as camview
import mods.comparator as comparator
import mods.timing
import objectmodel
import objectmatcher

# Because of OpenGL, siftgpu should only be used in one thread.
import siftgpu

T = mods.timing.TimerOff()

class CFeatureExtractor:
    def __init__(self):
        self._sift = None

    @property
    def SIFT(self):
        if self._sift == None:
            self._sift = siftgpu.SiftGPU(params="-s")
        return self._sift

    # Image is an RGB nparray
    def extractFeatures(self, image):
        self.SIFT.RunSIFT(image)
        k, d = self.SIFT.GetFeatureVector()
        if k == None or d == None: return None
        return camview.CFeaturepack(k, d)

class CModelManager:
    def __init__(self):
        self.models = {} # (filename, model)

    @property
    def modelNames(self):
        return sorted([model[1] for model in self.models.iterkeys()])

    # Model filenames:
    #   NAME.model.dat.gz: a file with the optimized model
    #   NAME.view.XXXX.dat.gz: a file with data for one view
    def addModel(self, name, directory):
        filename = "%s/%s.model.dat.gz" % (directory, name)
        if not os.path.exists(filename):
            raise IOError("File does not exist: '%s'" % filename)
        self.models[name] = [None, name, directory]
        pass

    def _loadModel(self, model):
        name, mdldir = model[1], model[2]
        m = objectmodel.CObjectModel(name)
        m.loadViews(mdldir, name, delayedLoad=True)
        m.loadPairMatches(mdldir, name)
        model[0] = m

    def getModel(self, name):
        if not self.models.has_key(name): return None
        model = self.models[name]
        if model[0] == None: self._loadModel(model)
        return model[0]

Manager = CModelManager()
Extractor = CFeatureExtractor()
Matcher = objectmatcher.CObjectMatcher()
def findMatchingObject(image):
    print "Image shape", image.shape
    if len(Manager.modelNames) < 1:
        print "No model loaded"
        return []
    exmpl = Extractor.extractFeatures(image)
    scores = []
    for mn in Manager.modelNames:
        model = Manager.getModel(mn)
        Matcher.match(model, exmpl)
        sc = Matcher.scores[0]
        sclimit = (5.0, 20.0)
        if sc < sclimit[1]:
            prob = 1.0 - (sc - sclimit[0]) / sclimit[1]
            if prob > 1: prob = 1.0
            if prob < 0: prob = 0.0
            scores.append(Matcher.bestviews[0].id)
            scores.append(prob)
    return scores

