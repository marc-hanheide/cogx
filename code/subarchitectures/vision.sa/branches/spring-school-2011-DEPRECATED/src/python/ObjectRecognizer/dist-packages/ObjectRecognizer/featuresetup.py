#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jul 2009 

import os, sys
import traceback
import numpy as np

import mods.cameraview as camview
import osmods.sift

import logging
LOG = logging.getLogger("ObjectRecognizer")

#class MyLogger():
#    def debug(self, msg): print "D", msg
#    def info(self, msg): print "I", msg
#LOG = MyLogger()

try:
    import siftgpu
except:
    exceptionType, exceptionValue, exceptionTraceback = sys.exc_info()
    traceback.print_exception(exceptionType, exceptionValue, exceptionTraceback)
    LOG.error("**** ObjectRecognizer: SIFTGPU will not be available")

try:
    import siftcuda
except:
    exceptionType, exceptionValue, exceptionTraceback = sys.exc_info()
    traceback.print_exception(exceptionType, exceptionValue, exceptionTraceback)
    LOG.error("**** ObjectRecognizer: SIFTCUDA will not be available")

def verifyImage(image):
    im = image[:960,:1280] # size limit
    # SiftGPU should support RGB, but it doesn't always work. Convert to GS.
    if len(im.shape) > 2 and im.shape[2] == 3:
        from PIL import Image
        resultImage = Image.fromarray(im)
        resultImage = resultImage.convert("L")
        resultImage.save('/tmp/sift_inputimage.jpg')
        im = np.asarray(resultImage)
    return im

class CFeatureExtractor:
    # Processes a RGB image and returns a CFeaturepack with the extracted features
    def extractFeatures(self, image):
        assert(False)
        return None

class CFeatureExtractorGpuBoost(CFeatureExtractor):
    def __init__(self):
        self._sift = None

    @property
    def SIFT(self):
        if self._sift == None:
            self._sift = siftgpu.SiftGPU(params="-s")
        return self._sift

    # Image is an RGB or GS nparray
    def extractFeatures(self, image):
        im = verifyImage(image)
        self.SIFT.RunSIFT(im)
        k, d = self.SIFT.GetFeatureVector()
        if k == None or d == None: return None
        return camview.CFeaturepack(k, d)

class CFeatureExtractorGpu(CFeatureExtractor):
    # Image is an RGB or GS nparray
    def extractFeatures(self, image):
        im = verifyImage(image)
        k,d = siftgpu.extractFeatures(im)
        if k == None or d == None: return None
        return camview.CFeaturepack(k, d)

class CFeatureExtractorNumpy(CFeatureExtractor):
    def extractFeatures(self, image):
        if len(image.shape) > 2 and len(image.shape[2] == 3):
            image = ( 2 * image[:,:,0]  + 6 * image[:,:,1] + 2 * image[:,:,2] ) / 10
        rows, cols = image.shape[:2]
        f = open("__tmp.pgm", "w")
        f.write('P5\n%d\n%d\n255\n' % (cols, rows))
        image.write(f)
        f.close()
        os.system("./output/bin/sift <__tmp.pgm >__sift.dat")
        #row, col, scale, orientation of each feature
        k, d = osmods.sift.read_features_from_file("__sift.dat")
        return camview.CFeaturepack(k, d)

class CDescriptorMatcher(object):
    # Match features from 2 lists, sort by distance and retain the ones with
    # an acceptable best-to-second-best ratio (1 - all are acceptable, 0 - nothing is acceptable)
    # @param maxRatio 
    #       if Fa1 has nearest matches Fb2 and Fb3 at distances d2 and d3, the match is rejected
    #       when d2/d3 > maxRatio.
    # Returns: List of lists:
    #     [ (i, [(d, j), ...]), ...]
    #     i - index of feature from list1
    #     j - index of feature from list2
    #     d - distance between two features
    def matchLists(self, list1, list2, maxRatio=1.0):
        assert(False)
        return None

    # Match the two lists of descriptors and calculate a homography.
    # return the list of descriptor pairs that fit the calculated homography.
    def homographyMatchLists(self, list1, list2, maxRatio=1.0):
        assert(False)
        return None

def distance(f1, f2):
    diff = f2 - f1 # subtract f from every row
    sqdiff = diff*diff
    # sum/reduce each row
    diff = sqdiff.sum()
    return np.sqrt(diff)

# Compare two arrays of feature descriptors with numpy
class CDescriptorMatcherNumpy(CDescriptorMatcher):
    def __init__(self, nBest=2, best=min):
        self.nBest = nBest
        self.order = 0 if best == min else 1
        pass

    # all euclidean distances from feature to rows of an array
    def distancesA(self, f, array1):
        diff = array1 - f # subtract f from every row
        sqdiff = diff*diff
        # sum/reduce each row
        diff = sqdiff.sum(axis=1)
        return np.sqrt(diff)

    # Match feature f to all features (rows) in array1 and sort by distance
    def matchA(self, f, array1):
        dists = self.distancesA(f, array1)
        iidx = dists.argsort()
        if self.nBest < 1: return [(dists[i], i) for i in iidx]
        if self.order == 0:
            return [(dists[i], i) for i in iidx[:self.nBest]]
        else:
            subidx = iidx[-self.nBest:]; subidx.reverse()
            return [(dists[i], i) for i in subidx]

    def matchLists(self, list1, list2, maxRatio=1.0):
        if maxRatio > 1.0: maxRatio = 1.0
        if maxRatio <= 0.0: return []
        #if isinstance(list2, np.ndarray) and (isinstance(list1, np.ndarray) or isinstance(list1[0], np.ndarray)):
        #    fnMatch = self.matchA
        #else: fnMatch = self.matchL
        fnMatch = self.matchA
        dists = []
        for i, f in enumerate(list1):
            cool = fnMatch(f, list2)
            if maxRatio < 1.0 and len(cool) > 1:
                if cool[1][0] > 0 and cool[0][0] / cool[1][0] > maxRatio: cool = None
            if cool != None and len(cool) > 0:
                dists.append( (i, cool) )
        return dists

class CDescriptorMatcherGpuBoost(CDescriptorMatcher):
    def __init__(self, nBest=2, best=min):
        self.nBest = nBest
        self.order = 0 if best == min else 1
        self._siftmatch = None

    @property
    def MATCH(self):
        if self._siftmatch == None:
            self._siftmatch = siftgpu.SiftMatchGPU(max_sift=4096)
        return self._siftmatch

    def matchLists(self, list1, list2, maxRatio=1.0):
        if maxRatio > 1.0: maxRatio = 1.0
        if maxRatio <= 0.0: return []
        iidx = self.MATCH.GetSiftMatch(list1, list2)
        result = [
            (p[0], [( distance(list1[p[0]], list2[p[1]]), p[1])])
            for p in iidx
            ]
        return result

class CDescriptorMatcherGpu(CDescriptorMatcher):
    def __init__(self, nBest=2, best=min):
        self.nBest = nBest
        self.order = 0 if best == min else 1

    def matchLists(self, list1, list2, maxRatio=1.0):
        if maxRatio > 1.0: maxRatio = 1.0
        if maxRatio <= 0.0: return []
        iidx = siftgpu.matchDescriptors(list1, list2)
        result = [
            (p[0], [( distance(list1[p[0]], list2[p[1]]), p[1])])
            for p in iidx if p[1] >= 0
            ]
        return result

class CDescriptorMatcherCuda(CDescriptorMatcher):
    def __init__(self, nBest=2, best=min):
        self.nBest = nBest
        self.order = 0 if best == min else 1

    def matchLists_batch(self, list1, list2, maxRatio=1.0):
        if maxRatio > 1.0: maxRatio = 1.0
        if maxRatio <= 0.0: return []
        result = []
        batch = 512
        nloop = list1.shape[0] / batch # limited gpu memory
        for i in xrange(nloop):
            iidx = siftcuda.matchDescriptors(list1[i*batch:i*batch+batch], list2)
            r = [
                (p[0], [( distance(list1[p[0]], list2[p[1]]), p[1])] )
                for p in iidx if p[1] != 0
                ]
            result.extend(r)
        return result

    def matchLists(self, list1, list2, maxRatio=1.0):
        if maxRatio > 1.0: maxRatio = 1.0
        if maxRatio <= 0.0: return []
        iidx = siftcuda.matchDescriptors(list1, list2)
        result = [
            (p[0], [( distance(list1[p[0]], list2[p[1]]), p[1])] ) # this adds 10% to the total time
            # (p[0], [( p[2], p[1] )] ) # the scores in p[2] (calc. by siftcuda) are not distances!
            for p in iidx if p[1] != 0
            ]
        return result

    def homographyMatchLists(self, list1, list2, maxRatio=1.0):
        res = siftcuda.homographyMatchDescriptors(list1, list2)
        if res == None:
            siftMatch = []
            hmgrMatch = []
            homography = None
        else:
            (siftIidx, hmgrIidx, homography) = res
            if siftIidx == None:
                siftMatch = []
                hmgrMatch = []
            else:
                siftMatch = [
                (p[0], [( p[3], p[1])])
                for p in siftIidx if p[1] >= 0
                ]
                if hmgrIidx == None: hmgrMatch = []
                else:
                    hmgrMatch = [
                    (p[0], [( distance(list1[p[0]], list2[p[1]]), p[1])])
                    for p in hmgrIidx if p[1] >= 0
                    ]
        return (siftMatch, hmgrMatch, homography)


class CSiftSetup:
    NUMPY = 0
    GPU = 1
    CUDA = 2
    def __init__(self, extractor, matcher):
        if extractor == CSiftSetup.NUMPY: self.extractor = CFeatureExtractorNumpy()
        else: self.extractor = CFeatureExtractorGpu()
        if matcher == CSiftSetup.NUMPY: self.matcher = CDescriptorMatcherNumpy(2, min)
        elif matcher == CSiftSetup.CUDA: self.matcher = CDescriptorMatcherCuda(2, min)
        else: self.matcher = CDescriptorMatcherGpu(2, min)

    def __str__(self):
        return "CSiftSetup %s, %s" % (self.extractor, self.matcher)

