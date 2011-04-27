#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: march 2009 
import os, sys
import gzip, pickle
#import opencv as cv
#import opencv.highgui as hg
#import opencv.adaptors as cvada
import cv
import numpy as np
import scipy as sci
import scipy.stats as stat
import numutil
import segmentor

def DEBUG(msg): pass
def debugOn():
    def f(msg): print msg
    global DEBUG; DEBUG=f
def debugOff():
    def f(msg): pass
    global DEBUG; DEBUG=f

# A set of fetures acquired from a single frame
# Keypoints and descriptors are stored in two numpy ndarrays
class CFeaturepack(object):
    def __init__(self, keypoints, descriptors):
        self.id = 0
        self.keypoints = keypoints
        self.descriptors = descriptors
        assert (len(self.keypoints) == len(self.descriptors))

    def __len__(self):
        return min(len(self.keypoints), len(self.descriptors))

    def __getitem__(self, i):
        if i < 0 or i >= len(self): raise IndexError
        return CFeatureInstance(self, i)

# A feature in a featurepack
# TODO: This could probably be implemented with a rsarray
class CFeatureInstance(object):
    def __init__(self, featurepack, index):
        self.featurepack = featurepack
        self.index = index

    def keypoint(self):
        return self.featurepack.keypoints[self.index]

    def descriptor(self):
        return self.featurepack.descriptors[self.index]

    def descrDistance(self, feature):
        if feature is CFeatureInstance:
            c = self.descriptor() - feature.descriptor()
        else: # Assume it's a 128-element array - descriptor
            c = self.descriptor() - feature
        return np.sqrt(np.dot(c,c)) # euclidean distance

    @property
    def x(self):
        return self.keypoint()[0]

    @property
    def y(self):
        return self.keypoint()[1]

    @property
    def scale(self):
        return self.keypoint()[2]

    @property
    def orientation(self):
        return self.keypoint()[3]

    @property
    def location(self):
        return self.keypoint()[:2]

    def distanceTo(self, feature):
        if feature is CFeatureInstance:
            c = self.location - feature.location
        else: # Assume it's a 2-element array - location
            c = self.location - feature
        return np.sqrt(np.dot(c,c)) # euclidean distance


# A group of features with a similar descriptor
class CFeatureGroup(object):
    def __init__(self, firstFeature = None):
        self.features = []
        self.__cache_loc = None
        self.__cache_dscr = None
        self.aveLocation = None
        self.aveDescriptor = None # Average descriptor
        self.weightDescriptor = None # quality of each dimension in aveDescriptor
        def _weighter(x):
            if x <= 0.003: return 1.0
            else: return 1 / (1 + 30 * (x - 0.003)) # decrease with distance
        self.fnWeighter = _weighter # weight calculator f(x)->[0..1], where x is stddev
        if firstFeature != None: self.addFeature(firstFeature)

    # join all descriptors into a ndarray 
    def descriptorArray(self):
        if self.__cache_dscr == None:
            self.__cache_dscr = np.array([f.descriptor() for f in self.features])
        return self.__cache_dscr

    # join all locations into a ndarray
    def locationArray(self):
        if self.__cache_loc == None:
            self.__cache_loc = np.array([f.location for f in self.features])
        return self.__cache_loc

    def addFeature(self, f):
        self.features.append(f)
        self.__cache_loc = None
        self.__cache_dscr = None
        self._updateAverage()

    def _updateAverage(self):
        self.aveLocation = np.average(self.locationArray(), axis=0)
        self.aveDescriptor = np.average(self.descriptorArray(), axis=0)
        dev = np.std(self.descriptorArray(), axis=0)
        self.weightDescriptor = [self.fnWeighter(x) for x in dev]

#class CImagePack(object):
#    def __init__(self):
#        self.image = None
#        self.imageDiffs = None

# A set of feature packs from a single camera configuration
# A feature pack is a tuple ( keypoints, descriptors )
class CViewPoint(object):
    def __init__(self):
        self._id = None
        self.vpPhi = None # lattitude
        self.vpLambda = None # longitude
        self.featurePacks = []
        self._image = None
        self._imagepath = None    # from where to load the image
        self.imageSize = None     # To restore original size from preview
        self._preview = None
        self._previewpath = None  # from where to load the preview
        self._mask = None

    def checkTypes(self):
        assert(isinstance(self.featurePacks, list))
        for fp in self.featurePacks:
            assert(isinstance(fp, CFeaturepack))
            assert(isinstance(fp.keypoints, np.ndarray))
            assert(isinstance(fp.descriptors, np.ndarray))
    
    # keypoints, feature descriptors
    def addMeasurements(self, keypoints, descriptors):
        self.featurePacks.append( CFeaturepack(keypoints, descriptors) )
    
    @property
    def image(self):
        if self._image != None: return self._image
        if self._imagepath != None:
            self.loadImage(self._imagepath)
            self._imagepath = None

    @property
    def preview(self):
        if self._preview != None: return self._preview
        if self._previewpath != None:
            self.loadPreview(self._previewpath)
            self._previewpath = None

    def setImage(self, image):
        self._image = image
        if image != None: self.imageSize = (image.width, image.height)
        self._mask = None

    def clearAllFeatures(self):
        self.featurePacks = []

    def save(self, filename):
        assert(filename.endswith(".dat.gz"))
        basename = filename[:-7]
        mask = self._mask
        if mask != None:
            mask = np.array(cvada.Ipl2NumPy(mask))
            # mask = cvada.Ipl2NumPy(mask) # WARNING Numeric.array !!!!
        preview = self.preview
        if preview != None:
            preview = np.array(cvada.Ipl2NumPy(preview))
            # preview = cvada.Ipl2NumPy(preview) # WARNING Numeric.array !!!!

        pickleDict = {
            "ID": self._id,
            "FeaturePacks": self.featurePacks,
            "ImageSize": self.imageSize,
            "Lambda": self.vpLambda,
            "Phi": self.vpPhi
            # "Preview": preview,
            # "Mask": mask
        }
        print "Saving ", filename, self.vpLambda, self.vpPhi
        stream = gzip.open(filename, "w")
        pickle.dump(pickleDict, stream)

    def load(self, filename):
        assert(filename.endswith(".dat.gz"))
        basename = filename[:-7]
        DEBUG(filename)

        stream = gzip.open(filename, "r")
        pickleDict = pickle.load(stream)
        def restore(key, default):
            if not pickleDict.has_key(key): return default
            return pickleDict[key]
        self._id = restore("ID", "")
        self.featurePacks = restore("FeaturePacks", [])
        self.imageSize = restore("ImageSize", None)
        self.vpLambda = restore("Lambda", None)
        self.vpPhi = restore("Phi", None)

        #self.preview = restore("Preview", None)
        #self._mask = restore("Mask", None)
        #print type(self.preview), self.preview.size, self.preview.shape
        #if self.preview != None:
        #    pass
        #    NONE OF THESE WORKS: 
        #    import ctypecv.interfaces as intrf
        #    # self.preview = cvada.NumPy2Ipl(Numeric.array(self.preview))
        #    # self.preview = intrf.cvCreateImageFromNumpyArray(self.preview)
        #    # self.preview = intrf.cvCreateMatNDFromNumpyArray(self.preview)
        #if self._mask != None: self._mask = cvada.NumPy2Ipl(self._mask)

    def loadImage(self, filename):
        if os.path.exists(filename):
            img = cv.LoadImage(filename)
            self.setImage(img)

    def loadPreview(self, filename):
        if os.path.exists(filename):
            self._preview = cv.LoadImage(filename)

    def savePreview(self, filename):
        if self.preview != None:
            pass # FIXME: opencv SVN2034 bug, hg.cvSaveImage(filename, self.preview)

    def createPreview(self, maxSize = 200):
        if maxSize > 400: maxSize = 400
        if self.image == None:
            return # TODO Notify error
        image = self.image
        if max(image.width, image.height) <= maxSize:
            self._preview = capture.copyFrame(image, copyData=True)
        else:
            cx = 1.0 * maxSize / image.width
            cy = 1.0 * maxSize / image.height
            scale = min(cx, cy)
            cx = int(image.width * scale)
            cy = int(image.height * scale)
            self._preview = cv.cvCreateImage(cv.cvSize(cx, cy), image.depth, image.nChannels)
            cv.cvResize(image, self._preview)

    @property
    def mask(self):
        if self.image == None: return None
        if self._mask != None: return self._mask
        sg = segmentor.CSegmentor()
        self._mask = sg.getObjectRegion(self.image)
        return self._mask

    def isFeatureInMask(self, feature):
        if self.mask == None: return True # if no mask, every feature is ok
        cx = self.imageSize[0] / self.mask.width
        cy = self.imageSize[1] / self.mask.height
        x = int(feature.x / cx + 0.5)
        y = int(feature.y / cy + 0.5)
        if self.mask[y, x] != 0: return True


class CViewPointFeatureEvaluator(object):
    def __init__(self, viewPoint):
        self._viewPoint = viewPoint
        self.stableFeatures = []
        self.unstableFeatures = []
        self.lonelyFeatures = []

    @property
    def image(self):
        if self._viewPoint == None: return None
        return self._viewPoint.image

    def filter1(self):
        # find nearest group by location; then check if similarity is ok
        def _tryadd(f, G):
            if len(G) > 0:
                gmin = G[0]
                gmindist = f.distanceTo(gmin.aveLocation)
                for fgrp in G:
                    if f.distanceTo(fgrp.aveLocation) < gmindist and f.descrDistance(fgrp.aveDescriptor) < 0.1:
                        gmin = fgrp
                        gmindist = f.distanceTo(fgrp.aveLocation)
                if gmindist < 2: # less than 2 pixels
                    gmin.addFeature(f)
                    return
            # No suitable group was found
            fgrp = CFeatureGroup()
            fgrp.addFeature(f)
            G.append(fgrp)
            return

        groups = []
        for pack in self._viewPoint.featurePacks:
            for feature in pack:
                _tryadd(feature, groups)
        return groups

    def filter2(self):
        # find nearest group by (average) descriptor; then check if distance is OK
        def _tryadd(f, G):
            if len(G) > 0:
                gmin = G[0]
                gmindist = f.descrDistance(gmin.aveDescriptor)
                for fgrp in G:
                    if f.descrDistance(fgrp.aveDescriptor) < gmindist and f.distanceTo(fgrp.aveLocation) <= 2:
                        gmin = fgrp
                        gmindist = f.descrDistance(fgrp.aveDescriptor)
                if gmindist < 0.1:
                    gmin.addFeature(f)
                    return
            # No suitable group was found
            fgrp = CFeatureGroup()
            fgrp.addFeature(f)
            G.append(fgrp)
            return

        groups = []
        for pack in self._viewPoint.featurePacks:
            for feature in pack:
                _tryadd(feature, groups)
        return groups

    # This system seems to work better than comparing with average descriptors:
    #   1. find the group with an item 
    #      with a descriptor that is nearest to the current descriptor
    #      where the distance between keypoints is within 2 pixels
    #   2. add to group if min distance is less than some threshold (0.1)
    def filter3(self):
        # find nearest group by descriptor; then check if distance is OK
        def _tryadd(f, G):
            if len(G) > 0:
                gmin = G[0]
                gmindist = min([f.descrDistance(gf.descriptor()) for gf in gmin.features])
                for fgrp in G:
                    dist = min([f.descrDistance(gf.descriptor()) for gf in fgrp.features])
                    if dist < gmindist and f.distanceTo(fgrp.aveLocation) <= 2:
                        gmin = fgrp
                        gmindist = dist
                if gmindist < 0.1:
                    gmin.addFeature(f)
                    return
            # No suitable group was found
            fgrp = CFeatureGroup()
            fgrp.addFeature(f)
            G.append(fgrp)
            return

        groups = []
        for pack in self._viewPoint.featurePacks:
            for feature in pack:
                if self._viewPoint.isFeatureInMask(feature): _tryadd(feature, groups)
        return groups


    # same as filter 4, but groups initialized to the largest featurepack
    # probably the most correct one
    def filter3a(self):
        def _tryadd(f, G):
            if len(G) > 0:
                gmin = G[0]
                gmindist = min([f.descrDistance(gf.descriptor()) for gf in gmin.features])
                for fgrp in G:
                    dist = min([f.descrDistance(gf.descriptor()) for gf in fgrp.features])
                    if dist < gmindist and f.distanceTo(fgrp.aveLocation) <= 2:
                        gmin = fgrp
                        gmindist = dist
                if gmindist < 0.1:
                    gmin.addFeature(f)
                    return
            # No suitable group was found
            fgrp = CFeatureGroup()
            fgrp.addFeature(f)
            G.append(fgrp)
            return

        longpack = self._viewPoint.featurePacks[0]
        for pack in self._viewPoint.featurePacks:
            if len(pack) > len(longpack): longpack = pack
        groups = [CFeatureGroup(f) for f in longpack]
        for pack in self._viewPoint.featurePacks:
            if pack == longpack: continue
            for feature in pack:
                _tryadd(feature, groups)
        return groups

    def optimize(self, stable = 4):
        groups = self.filter3()
        self.stableFeatures = [g for g in groups if len(g.features) >= stable]
        self.unstableFeatures = [g for g in groups if len(g.features) < stable and len(g.features) > 1]
        self.lonelyFeatures = [g for g in groups if len(g.features) == 1]

from pylab import plot, figure, imshow, xlabel, ylabel, cm, show, suptitle
def showgroups(groups):
    figure (figsize=(10,5))
    xlabel("SIFT vector element")
    ylabel("Deviation")
    dev = None
    sms = 0; nftr = 0; ngrp = 0
    for grp in groups:
        dev = np.std(grp.descriptorArray(), axis=0)
        for s in dev: sms += s
        ngrp += 1; nftr += len(grp.features)
        plot(dev)
    suptitle("%d features, %d/%d groups, sum(stddev)=%f" % (nftr, ngrp, len(groups), sms))
    show()

import itertools
def checkClassificationW(view):
    # Check Mahalanobis (suggested by AŠ)
    nerr = 0
    for grp in view.stableFeatures:
        for ftr in grp.features:
            bestg = grp
            bestdist = numutil.siftdistancew(ftr.descriptor(), grp.aveDescriptor, grp.weightDescriptor)
            for g1 in itertools.chain(view.stableFeatures, view.unstableFeatures, view.lonelyFeatures):
                dist = numutil.siftdistancew(ftr.descriptor(), g1.aveDescriptor, g1.weightDescriptor)
                if dist < bestdist:
                    bestdist = dist
                    bestg = g1
                    nerr += 1

    print "Classified to another group (weighted): %d" % nerr

def checkClassification(view):
    nerr = 0
    for grp in view.stableFeatures:
        for ftr in grp.features:
            bestg = grp
            bestdist = numutil.siftdistance(ftr.descriptor(), grp.aveDescriptor)
            for g1 in itertools.chain(view.stableFeatures, view.unstableFeatures, view.lonelyFeatures):
                dist = numutil.siftdistance(ftr.descriptor(), g1.aveDescriptor)
                if dist < bestdist:
                    bestdist = dist
                    bestg = g1
                    nerr += 1

    print "Classified to another group: %d" % nerr
