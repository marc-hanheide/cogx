#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009 

import os, sys
import re
import gzip, pickle
#import opencv as cv
#import opencv.highgui as hg
import cv
import numpy as np

import mods.cameraview as camview
import mods.comparator as comparator
import mods.timing

T = mods.timing.TimerOn()

def coordinateGrid(sizeHalf=1, center=(0, 0), addCenter=False):
    cells = []
    for x in xrange(sizeHalf * 2 + 1):
        for y in xrange(sizeHalf * 2 + 1):
            if addCenter or x != sizeHalf or y != sizeHalf:
                cells.append( (center[0] + x-sizeHalf, center[1] + y-sizeHalf) )
    return cells

class CLateViewpointLoader(list):
    def __init__(self, basepath):
        self.basepath = basepath

    def __getitem__(self, i):
        item = list.__getitem__(self, i)
        if not isinstance(item, camview.CViewPoint):
            item = self.loadItem(i)
        return item

    # Encapsulate the default iterator in a new iterator that
    # will load views that are not yet loaded
    def __iter__(list_self):
        class _iterload: # CHECK may need to override/proxy other methods!
            def __init__(self, olditer):
                self.olditer = olditer
            def next(self):
                item = self.olditer.next()
                if not isinstance(item, camview.CViewPoint):
                    i = len(list_self) - self.olditer.__length_hint__() - 1
                    item = list_self.loadItem(i)
                    # print "ToDo: need to load this item from list_self...", i
                return item
        return _iterload(list.__iter__(list_self))

    def loadItem(self, index):
        item = list.__getitem__(self, index)
        if item == None: vpname = vpname = "%s%04d" % (self.basepath, index)
        else: vpname = "%s%s" % (self.basepath, item)
        item = camview.CViewPoint()
        item.load(vpname)
        item.loadImages(vpname)
        self[index] = item
        return item

# Matching features between 2 views of an object
class CMatchingFeatures(object):
    def __init__(self, vp1, vp2, matches):
        self.vpid1 = vp1.id
        self.vpid2 = vp2.id
        self.matches = self._convertToNdarray(matches)
        
    # feature_id feature2_1_id feature2_2_id dist2_1 dist2_2
    def _convertToNdarray(self, matches):
        if isinstance(matches, np.ndarray): return matches
        count = len(matches) - len( [ m for m in matches if len(m[1]) < 2] )
        arr = np.zeros((count, 5), np.single) # 32-bit float array
        for i, m in enumerate(matches):
            if len(m[1]) < 2: continue
            arr[i][0] = m[0] # id
            arr[i][1] = m[1][0][1] # id
            arr[i][2] = m[1][1][1] # id
            arr[i][3] = m[1][0][0] # dist
            arr[i][4] = m[1][1][0] # dist
        return arr


# Directory structure for models:
# .../root/
#   /ModelName
#       /image/
#           /ModelName_VPxxx_yyyy.png
#       /features/
#           /ModelName.view.nnnn.dat.gz
#           /ModelName.model.dat.gz
#       /preview/
#           /ModelName.view.nnnn.png
class CModelFileManager(object):
    def __init__(self, model, modelStorePath="/tmp"):
        self.model = model
        self.rootpath = modelStorePath

    def setModelStorePath(self, modelStorePath):
        self.rootpath = modelStorePath

    @property
    def imageDir(self):
        return "%s/%s/image" % (self.rootpath, self.model.name)

    @property
    def previewDir(self):
        return "%s/%s/preview" % (self.rootpath, self.model.name)

    @property
    def featureDir(self):
        return "%s/%s/features" % (self.rootpath, self.model.name)

    def checkModelDirs(self, create=True):
        def _checkDir(path):
            if os.path.exists(path): return True
            if create:
                os.makedirs(path)
            return os.path.exists(path)

        all = _checkDir(self.imageDir)
        all = all and _checkDir(self.previewDir)
        all = all and _checkDir(self.featureDir)
        return all

    #def _vpname(self, vp, index=-1):
    #    if index < 0: return "%s_VP%03d_L%03d" % (vp.vpPhi, vp.vpLambda)
    #    else: return "%s_VP%03d_I%03d" % (vp.vpPhi, index)
    def viewpointBase(self, viewpoint):
        return "%s_VP%03d_L%03d" % (self.model.name, viewpoint.vpPhi, viewpoint.vpLambda)

    def viewpointBase2(self, phi, lmbda):
        return "%s_VP%03d_L%03d" % (self.model.name, phi, lmbda)

    def parseNamePhiLambda(self, filename):
        filename = os.path.basename(filename)
        reId = re.compile(r"([a-z0-9]+)_VP([0-9]+)_L([0-9]+)\.", re.IGNORECASE)
        mo = reId.search(filename)
        if mo != None: return ( int(mo.group(1)), int(mo.group(2)) , int(mo.group(3)) )
        return None

    def parsePhiLambda(self, filename):
        filename = os.path.basename(filename)
        reId = re.compile(r"[a-z0-9]+_VP([0-9]+)_L([0-9]+)\.", re.IGNORECASE)
        mo = reId.search(filename)
        if mo != None: return ( int(mo.group(1)), int(mo.group(2)) )
        return None

    def parsePhiIndex(self, filename):
        filename = os.path.basename(filename)
        reId = re.compile(r"[a-z0-9]+_VP([0-9]+)_I([0-9]+)\.", re.IGNORECASE)
        mo = reId.search(filename)
        if mo != None: return ( int(mo.group(1)), int(mo.group(2)) )
        return None

    def vpFeaturePath(self, viewpoint):
        base = self.viewpointBase(viewpoint)
        return "%s/%s.view.dat.gz" % (self.featureDir, base)

    def vpPreviewPath(self, viewpoint):
        base = self.viewpointBase(viewpoint)
        return "%s/%s.jpg" % (self.previewDir, base)

    def vpImagePath(self, viewpoint):
        base = self.viewpointBase(viewpoint)
        return "%s/%s.png" % (self.imageDir, base)


class CObjectModel(object):
    def __init__(self, name):
        self.name = name
        self.viewPoints = []
        self.viewPairMatches = []
        self.FM = CModelFileManager(self)

    def saveViews(self):
        for i, vp in enumerate(self.viewPoints):
            # TODO vpname = "%s/%s.view.%04d" % (dirname, basename, i)
            if isinstance(vp, camview.CViewPoint): # will not be saved if it wasn't loaded
                fname = self.FM.vpFeaturePath(vp)
                vp.save(fname)
                fname = self.FM.vpPreviewPath(vp)
                # vp.savePreview(fname)
                # fname = self.FM.imagePath(vp)
                # vp.saveImage(fname)

    def __listViewpointFiles(self, dirname):
        files = [
            fn for fn in os.listdir(dirname) if fn.endswith(".view.dat.gz")
        ]
        files.sort()
        return files

    def loadViews(self, delayedLoad=False):
        delayedLoad = False # FIXME: delayedLoad NEEDS TO BE CHANGED
        if delayedLoad: self.viewPoints = CLateViewpointLoader(self.FM.featureDir + "/")
        else: self.viewPoints = []
        files = self.__listViewpointFiles(self.FM.featureDir)
        for fn in files:
            if delayedLoad: self.viewPoints.append(fn[:-7])
            else:
                vp = camview.CViewPoint()
                vp.load(self.FM.featureDir + "/" + fn)
                print os.path.basename(fn), vp.vpPhi, vp.vpLambda
                fnprev = self.FM.vpPreviewPath(vp)
                if os.path.exists(fnprev): vp._previewpath = fnprev
                fnimage = self.FM.vpImagePath(vp)
                if os.path.exists(fnimage):
                    vp._imagepath = fnimage
                    if vp.vpPhi == None or vp.vpLambda == None:
                        phil = self.FM.parsePhiLambda(fnimage)
                        if phil != None:
                            vp.vpPhi = phil[0]
                            vp.vpLambda = phil[1]
                self.viewPoints.append(vp)

    def savePairMatches(self, dirname, basename):
        pickleDict = {
            "ViewPairMatches": self.viewPairMatches
        }
        # FIXME save model
        #stream = gzip.open("%s/%s.model.dat.gz" % (dirname, basename), "w")
        #pickle.dump(pickleDict, stream)
        #stream.close()

    def loadPairMatches(self, dirname, basename):
        self.viewPairMatches = []
        fn = "%s/%s.model.dat.gz" % (dirname, basename)
        if os.path.exists(fn):
            stream = gzip.open(fn, "r")
            pickleDict = pickle.load(stream)
            def restore(key, default):
                if not pickleDict.has_key(key): return default
                return pickleDict[key]
            self.viewPairMatches = restore("ViewPairMatches", [])

    # reId: regular expression that extracts longitude and lattitude
    # lambdaPhi: subgroup index in matched reId: (longitude, lattitude)
    # returns: grid, phis, lambdas
    #   - every row is a list for lattitude Phi
    #   - elements of list are viewpoints, sorted by Lambda
    # TODO: lambda and phi are part of CViewPoint - use them!
    def arrangeViewsFromId(self, reId=None, lambdaPhi=(1, 2)):
        if reId == None:
            # default: Curve_015_Rot_045 ( -> Curve_PHI_Rot_LAMBDA)
            reId = re.compile("[a-z]+_([0-9]+)_[a-z]+_([0-9]+)", re.IGNORECASE)
            lambdaPhi = (2, 1)
        lidx = []
        for i, vp in enumerate(self.viewPoints):
            # if not isinstance(vp, camview.CViewPoint): vp = self.viewPoints[i] # Workaround for late-loading
            mo = reId.search(vp.id)
            if mo != None: lidx.append( (i, int(mo.group(lambdaPhi[0])), int(mo.group(lambdaPhi[1]))) )
        grid = []
        if len(lidx) > 0:
            lams = set([ lf[1] for lf in lidx]); lams = sorted([i for i in lams])
            phis = set([ lf[2] for lf in lidx]); phis = sorted([i for i in phis])
            grid = [ [None for i in lams] for j in phis]
            for lf in lidx:
                l = lams.index(lf[1])
                p = phis.index(lf[2])
                grid[p][l] = self.viewPoints[ lf[0] ]
        return grid, phis, lams
    
    def calculateMatches_squareGrid(self, gridDist=1):
        grid, phis, lams = self.arrangeViewsFromId()
        self.viewPairMatches = []
        T.ntic(1)
        total = len(phis) * len(lams)
        for ip in xrange(len(phis)):
            for il in xrange(len(lams)):
                T.ntoc(1, "%d/%d started at" % (ip*len(lams)+il+1, total))
                v1 = grid[ip][il]
                if v1==None: continue
                print v1.id
                T.tic()
                fp1 = v1.featurePacks[0]
                toCheck = coordinateGrid(gridDist, center=(ip, il), addCenter=False)
                for cc in toCheck:
                    # prevent wrap-around; TODO some models may allow it
                    if cc[0] < 0 or cc[1] < 0: continue
                    if cc[0] >= len(phis) or cc[1] > len(lams): continue
                    try: v2 = grid[cc[0]][cc[1]]
                    except: continue # probably off limits
                    if v2 == None: continue
                    fp2 = v2.featurePacks[0]
                    C = comparator.CDescriptorMatcher()
                    dists = C.matchLists(fp1.descriptors, fp2.descriptors, maxRatio=0.8)
                    dists.sort(key=lambda x: x[1][0])
                    self.viewPairMatches.append(CMatchingFeatures(v1, v2, dists))
                T.toc()
    
    # Groups of views with the same phi; each group ordered by lambda
    def arrangeViewsByPhi(self):
        # for i, vp in enumerate(self.viewPoints):
        #    if not isinstance(vp, camview.CViewPoint): vp = self.viewPoints[i] # Workaround for late-loading
        phis = set( [vp.vpPhi for vp in self.viewPoints] ); phis = sorted( [i for i in phis] )
        levels = []
        for phi in phis:
            lev = [ vp for vp in self.viewPoints if vp.vpPhi == phi ]
            lev = sorted(lev, key=lambda x: x.vpLambda)
            levels.append(lev)
        return levels, phis

    # calculate matches between a view and it's nearest views
    def calculateMatches(self, phiDist=None, labmdaDist=None):
        def matchViews(v1, v2):
            fp1 = v1.featurePacks[0]
            fp2 = v2.featurePacks[0]
            C = comparator.CDescriptorMatcher()
            dists = C.matchLists(fp1.descriptors, fp2.descriptors, maxRatio=0.8)
            dists.sort(key=lambda x: x[1][0])
            return CMatchingFeatures(v1, v2, dists)

        def viewsAround(level, lmbda, labmdaDist=30):
            def angleDist(a, b):
                d = (a - b) % 360
                if d > 180: d = 360 - d
                return d
            return [
                v for v in level
                if angleDist(v.vpLambda, lmbda) < labmdaDist
            ]

        self.viewPairMatches = []
        levels, phis = self.arrangeViewsByPhi()
        # ATM: look at level above, below, max 30 deg. L/R; cur level: L/R neighbour
        T.tic()
        for il, lev in enumerate(levels):
            print il, "phi", phis[il], "count", len(lev), [l.vpLambda for l in lev]
            for iv, view in enumerate(lev):
                toCheck = []
                lr = [ (iv - 1) % len(lev), (iv + 1) % len(lev) ] # left, right
                toCheck.extend([ lev[i] for i in lr ])
                if il > 0: # level below
                    lr = viewsAround(levels[il-1], view.vpLambda)
                    toCheck.extend(lr)
                if il < len(levels)-1: # level above
                    lr = viewsAround(levels[il+1], view.vpLambda)
                    toCheck.extend(lr)
                print view.id, "to ckeck:", len(toCheck)
                for v2 in toCheck:
                    m = matchViews(view, v2)
                    self.viewPairMatches.append(m)
        dt = T.toc("Matches calculated in")
        if dt > 0 and len(self.viewPairMatches) > 0:
            print "%.4fs per view pair" % (dt / len(self.viewPairMatches)) # 0.6s

    def defineFeatureLevels(self):
        # a level is assigned to every feature. Start recognizng at max level
        # Level 0: all features
        # Level 1: features present in multiple neighbours
        # Level 2: eg. very distinctive features
        class CGoodFeatures:
            def __init__(self, viewPoint, indexList):
                self.viewPoint = viewPoint
                self.indices = sorted(indexList)
                self._keypoints = None
                self._descriptors = None

            @property
            def keypoints(self):
                if self._keypoints == None:
                    fp = self.viewPoint.featurePacks[0]
                    self._keypoints = fp.keypoints.take(self.indices, axis=0)
                return self._keypoints

            @property
            def descriptors(self):
                if self._descriptors == None:
                    fp = self.viewPoint.featurePacks[0]
                    self._descriptors = fp.descriptors.take(self.indices, axis=0)
                return self._descriptors

        goodFeatures = []
        cntall, cntgood = 0, 0
        for view in self.viewPoints:
            hist = {} # For each matched feature the number of nbr. views with a match
            matchedPairs = [ pm for pm in self.viewPairMatches if pm.vpid1 == view.id ]
            for pair in matchedPairs:
                cntall += len(pair.matches)
                for m in pair.matches: # rows of 5-element matrix, see _convertToNdarray
                    fidx = m[0] # feature (index) from current view
                    try: hist[fidx] += 1
                    except: hist[fidx] = 1
            good = [
                int(k) for k, v in hist.items()
                if v > len(matchedPairs) * 0.5
            ]
            cntgood += len(good)
            goodFeatures.append(CGoodFeatures(view, good))

        print cntgood, cntall, cntgood * 1.0 / cntall
        return goodFeatures

    def selectBestViews(self):
        # eg.: best view has the most matches with it's neighbours
        # border of view-grid is (most likely) not included: N*M -> (N-2)*(M-2)
        # select BV in each 8-neighbourhood / 4-N -> (N-2)*(M-2) / 4
        # maybe: remove views that are still very similar to nearest best views
        # maybe: learn some "cluttered" bgds. for negative examples
        pass

