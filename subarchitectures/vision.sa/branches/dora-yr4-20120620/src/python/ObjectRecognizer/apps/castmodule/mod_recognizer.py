#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009 

import os, sys, traceback, time
import numpy as np
import logging
import cast.pylog4cxx as pylog4cxx

try: import castinit
except: pass

import ObjectRecognizer.mods.cameraview as camview
import ObjectRecognizer.mods.comparator as comparator
import ObjectRecognizer.mods.timing as timing
import ObjectRecognizer.objectmodel as objectmodel
import ObjectRecognizer.objectmatcher as objectmatcher

from ObjectRecognizer.featuresetup import CSiftSetup
# NOTE: Because of OpenGL, siftgpu should only be used in one thread.

if 0:
    LOG = logging.getLogger("mod_ObjectRecognizer")
    LOG.setLevel(logging.DEBUG)
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)
    LOG.addHandler(ch)
else:
    pylog4cxx.configure()
    LOG = pylog4cxx.Logger("mod_recognizer", "vi.sa", "\033[34m")
    LOG.setLevel("trace")

T = timing.TimerOn()

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
        m.loadViews(delayedLoad=False)
        LOG.info("%ld views" % len(m.viewPoints))
        m.loadPairMatches(mdldir, name)

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
# Matcher = objectmatcher.CObjectDetector()
Manager = CModelManager()

def reconfigSift(extractor, matcher):
    global Setup
    LOG.info("Reconfig SIFT")
    Setup = CSiftSetup(extractor, matcher)
    print Setup

# Returns a list of tuples (model_name, lambda(yaw), phi(pitch), roll, probability)
def _findMatchingObject(features):
    global Setup, Matcher, Manager
    Matcher.descriptorMatcher = Setup.matcher
    matches = []
    names = []
    poses = []
    sclimit = (4.0, 20.0)
    for mn in Manager.modelNames:
        model = Manager.getModel(mn)
        T.tic();
        LOG.info("MATCH Model %s" % mn)
        Matcher.match(model, features) # Matcher.quickMatch(model, features)
        T.toc("Time to match:")
        LOG.info("%ld scores" % len(Matcher.scores))
        if len(Matcher.scores) < 1: continue
        names.append(mn)
        matches.append( (Matcher.bestViews[:3], Matcher.scores[:3], Matcher.consists[:3]) )
        pose = Matcher.getPoseProbability()
        pose.reverse()
        ap = np.ndarray(shape=(len(pose), 4), dtype=float)
        for i,p in enumerate(pose):
            for c in xrange(4): ap[i, c] = p[c] # %, phi, lambda, rot
        poses.append(ap)
        # printMatches(Matcher)

    def getProbs1(matches):
        LOG.info("Matching with estimation of parameter consistency (sift scale, rotaion)")
        probs = []
        sp = 0
        for m in matches:
            # H: expected best score; 0.3 matches; d0=0.001, max 10% better; decrease with distance
            dfac = m[2][0].scale * m[2][0].scaleConfidence
            if dfac > 1: dfac = 1
            if dfac <= 1e-3: dfac = 1e-3
            estsc = len(m[0][0].featurePacks[0]) * 0.3 * 100 * dfac
            realsc = m[1][0]
            probs.append( realsc / estsc ) # name, %, pose
            sp += realsc / estsc

        if sp > 0.9: # H: a "PDF" + unknown
            for p in probs: p = 0.9 * p / sp
        return probs

    def getProbs2(matches, nBestViews = 3):
        # H: PDF from rotation confidence
        LOG.info("Matching with estimation of homographies(ransac)")
        confs = []
        confmax = 0
        for m in matches:
            consists = m[2][:nBestViews] # use the desired number of consistencies
            if len(consists) == 0: confs.append(0)
            else:
                mconfs = [ c.rotationConfidence for c in consists ]
                cs = sum (mconfs)
                confs.append(cs / len(consists))
                confmax = max(confmax, max(mconfs))

        confs = [ (c - 0.3) / 0.7 for c in confs ]
        for i in xrange(len(confs)):
            if confs[i] < 0: confs[i] = 0
        sumconfs = sum(confs)
        if sumconfs < 1e-4: return confs
        if confmax >= 1.0: confmax = 0.999
        confs = [ c / sumconfs * confmax for c in confs ]
        return confs

    gpmode = 1
    if gpmode == 1: probs = getProbs1(matches)
    elif gpmode == 2: probs = getProbs2(matches)
    else: probs = [0]
    sp = sum(probs)
    if (sp < 1.0):
        names.append("*unknown*")
        probs.append(1.0 - sp)
        poses.append(None)

    # convert to something for CPP
    n = min(len(names), len(poses), len(probs))
    if len(names) != n or len(poses) != n or len(probs) != n:
        LOG.debug("_findMatchingObject: Inconsistent results")

    return (names, probs, poses)

def saveImage(ndarr, fname):
    from PIL import Image
    pilImage = Image.fromarray(ndarr, 'RGB') # RGB, RGBA, G ?
    pilImage.save(fname)

SaveImages = False
imid=0
# image: ndarray (h, w, 1 or 3)
# region: (x0, y0, x1, y1)
def findMatchingObject(image, region=None):
    global Setup, Matcher, Manager, SaveImages, imid
    try:
        matches = []
        LOG.info("Region: %s" % region)
        if len(Manager.modelNames) < 1:
            LOG.warnring("No model loaded")
            return ( ["*unknown*"], [1.0], [None] )
        if region != None:
            x0, x1, y0, y1 = region
            w = x1-x0; h = y1-y0
            if w > 0 and h > 0: image = np.copy(image[y0:y1, x0:x1])

        if SaveImages:
            imid += 1
            saveImage(image, "xdata/cast_objrecog_%04d.png" % imid)
        features = Setup.extractor.extractFeatures(image)
        matches = _findMatchingObject(features)
        fout = open('/tmp/or_model_distrib.html', 'w')
        fout.write("<table>")
        if len(matches) > 1:
            for i,m in enumerate(matches[0]):
                if i < len(matches[1]): fout.write("<tr><td>%s</td><td>%f</td></tr>" % (m, matches[1][i]))
                else: fout.write("<tr><td>%s</td><td>&nbsp;</td></tr>" % (m))
        elif len(matches) > 0: print matches[0]
        fout.write("</table>")
        fout.close()
        return matches
    except:
        exceptionType, exceptionValue, exceptionTraceback = sys.exc_info()
        traceback.print_exception(exceptionType, exceptionValue, exceptionTraceback)
        # traceback.print_tb(exceptionTraceback)
    return None

def testCppInterface(image, region=None):
    global Setup, Matcher, Manager
    try:
        LOG.info("testCppInterface (No processing done.)")
        LOG.info("Image shape %s:" % image.shape)
        if region != None:
            x0, y0, x1, y1 = region
            LOG.info("ROI (%d %d, %d %d) %d %d" % (x0, x1, y0, y1, x1-x0, y1-y0))
            image = np.copy(image[x0:x1, y0:y1])
            LOG.info("ROI shape %s" % image.shape)
        matches = ( ["A", "B", "C"], [0.7, 0.2, 0.1], [None, None, None] )
        return matches
    except:
        exceptionType, exceptionValue, exceptionTraceback = sys.exc_info()
        traceback.print_exception(exceptionType, exceptionValue, exceptionTraceback)
        # traceback.print_tb(exceptionTraceback)
    return None

def testMatching(image, region=None):
    global Setup, Matcher, Manager
    # Take first featurepack of first object and perform matching
    try:
        LOG.info("testMatching (No feature extraction.)")
        if len(Manager.modelNames) < 1:
            LOG.info("No model loaded")
            return ( ["*unknown*"], [1.0], [None] )
        mn = Manager.modelNames[0]
        model = Manager.getModel(mn)
        vp = model.viewPoints[0]
        ftrs = vp.featurePacks[0]
        matches = _findMatchingObject(ftrs)
        return matches
    except:
        exceptionType, exceptionValue, exceptionTraceback = sys.exc_info()
        traceback.print_exception(exceptionType, exceptionValue, exceptionTraceback)
        # traceback.print_tb(exceptionTraceback)
    return None
