#! /usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: may 2009 
import os, sys, time, pickle, itertools
import re
import gzip
import numpy as np
import opencv.cv as cv
import opencv.highgui as hg
import opencv.adaptors as cvada

import pymodulepaths
import siftgpu
from ObjectRecognizer.mods.capture import CameraCapture, FileCapture, copyFrame, CLoopback1394Capture
from ObjectRecognizer.mods.numutil import *
import ObjectRecognizer.mods.cameraview as camview
import ObjectRecognizer.mods.segmentor as segmentor
import ObjectRecognizer.objectmodel as model
import ObjectRecognizer.objectmatcher as matcher
import ObjectRecognizer.mods.comparator as comparator


# segmentor.SHOW_TIMES = True

class CModelBuilder:
    def __init__(self):
        self.model = None

    def _acquireSift(self, frame, smooth=False):
        if smooth:
            copy = cv.cvCreateImage(cv.cvSize(frame.width, frame.height), frame.depth, frame.nChannels)
            cv.cvSmooth(frame, copy, cv.CV_MEDIAN, 3, 3)
            frame = copy
        im = np.array(cvada.Ipl2NumPy(frame))
        k, d = siftgpu.extractFeatures(im)
        return (k, d)

    # A single vp can have multiple images (maybe to remove noise?)
    def createViewpoint(self, imageNameList, createPreview=True):
        vp = camview.CViewPoint()
        sgmt = segmentor.CSegmentor()
        pvimage = None
        for fname in imageNameList:
            image = hg.cvLoadImage(fname)
            if pvimage == None: pvimage = image
            print "Processing", fname
            # TODO: Process only segmented part of image (CSegmentor)
            # sgmt.getObjectRegion(image) !!! This takes 5-6s !!!
            keys, features = self._acquireSift(image, smooth=False)
            vp.addMeasurements(keys, features)
            phil = self.model.FM.parsePhiLambda(os.path.basename(fname))
            if phil != None:
                vp.vpPhi = phil[0]
                vp.vpLambda = phil[1]
        if createPreview and pvimage != None:
            vp.setImage(pvimage)
            vp.createPreview()
            vp._image = None

        return vp

        
    def buildModelViewpoints(self, amodel, indir, outdir):
        name = amodel.name
        self.model = amodel
        assert(len(name) > 0)
        files = []
        for fn in sorted(os.listdir(indir)):
            if fn.startswith(name + "_") and fn.endswith(".png"):
                files.append(fn)
        if len(files) < 1:
            print "No images for", name
            return
        print len(files), "images for", name
        for fn in files:
            vp = self.createViewpoint(["%s/%s" % (indir, fn)])
            vp.id = fn
            amodel.viewPoints.append(vp)
        amodel.saveViews()


    # Images were acquired in a regular grid (step in degrees)
    def createModelFromGrid(self, name, inprefix, outdir=None, step=15):
        omod = model.CObjectModel()
        self.model = omod
        ranges=[11, 11] # When Testing...
        ranges=[91, 181]
        for curve in xrange(0, ranges[0], step):
            for rot in xrange(0, ranges[1], step):
                fd = "Curve_%03d_Rot_%03d.png" % (curve, rot)
                fn = "%s__%s" % (inprefix, fd)
                vp = self.createViewpoint([fn])
                vp.id = fd
                omod.viewPoints.append(vp)
        if outdir != None: omod.save(outdir, name)
        return omod

def createModel_house():
    Builder = CModelBuilder()
    mdl = Builder.createModelFromGrid("House", "xdata/input/HouseL2Black/HouseL2Black", "xdata/models")
    # mdl.save("xdata/models", "House")
    # mdl = model.CObjectModel()
    # mdl.load("xdata/models", "House")
    # mdl.viewPoints[0].saveImages("xdata/models/House_test")

def processTestHouse():
    Builder = CModelBuilder()
    mdl = Builder.createModelFromGrid("HouseTest", "xdata/input/HouseL2Cluttered/HouseL2Cluttered", "xdata/testpreprocess", step=10)
    # mdl.save("xdata/testpreprocess", "HouseTest")

def createModel_twEarlGrey():
    builder = CModelBuilder()
    name = "TwEarlGrey"
    imdir = "../xdata/images/" + name
    mdldir = "../xdata/models/" + name
    amodel = model.CObjectModel(name)

    # process views only if processed data doesn't exist
    viewsBuilt = False
    for fn in sorted(os.listdir(mdldir)):
        if fn.startswith(name + ".view.") and fn.endswith(".dat.gz"):
            viewsBuilt = True
            break
    if viewsBuilt:
        print "Loading model"
        amodel.loadViews(mdldir, name)
        print len(amodel.viewPoints), "viewpoints loaded"
    else: builder.buildModelViewpoints(amodel, imdir, mdldir)

    # Assign lambda, phi to views
    # Phis are given in the filename; lambdas must be calculated from index (assume equidistant)
    reId = re.compile("[a-z]+_curve_([0-9]+)_([0-9]+)", re.IGNORECASE)
    lambdaPhi = (2, 1)
    lidx = []
    needsSaving = False
    for i, vp in enumerate(amodel.viewPoints):
        if not isinstance(vp, camview.CViewPoint): vp = amodel.viewPoints[i] # Workaround for late-loading
        mo = reId.search(vp.id)
        if mo != None: lidx.append( [ i, int(mo.group(lambdaPhi[0])), int(mo.group(lambdaPhi[1])) ] )
        if vp.vpLambda == None or vp.vpPhi == None: needsSaving = True
    phis = set([ lf[2] for lf in lidx]); phis = sorted([i for i in phis])

    for phi in phis:
        nv = 1 + max( [ lf[1] for lf in lidx if lf[2] == phi ] ) # max lambda-index for a given phi
        fac = 360.0 / nv
        for lf in lidx:
            if lf[2] == phi: lf[1] = int(lf[1] * fac)

    for lf in lidx:
        i = lf[0]
        amodel.viewPoints[i].vpLambda = lf[1]
        amodel.viewPoints[i].vpPhi = lf[2]
        print lf

    if needsSaving:
        print "Saving"
        amodel.saveViews(mdldir, name)
        needsSaving = False

    # Calculate some view interactins
    if not os.path.exists("%s/%s.model.dat.gz"):
        pass

def renameImageFiles(fileman):
    nPhiLam = 0
    nPhiInd = 0
    for fn in sorted(os.listdir(fileman.imageDir)):
        pair = fileman.parsePhiLambda(fn)
        if pair != None: nPhiLam += 1
        pair = fileman.parsePhiIndex(fn)
        if pair != None: nPhiInd += 1

    if nPhiInd == 0: return
    levels = {}
    for fn in sorted(os.listdir(fileman.imageDir)):
        sign = 1
        pair = fileman.parsePhiLambda(fn)
        if pair == None:
            pair = fileman.parsePhiIndex(fn)
            sign = -1
        if pair == None: continue
        phi = pair[0]; lam = pair[1]*sign
        if not levels.has_key(pair[0]): levels[phi] = [[fn, lam]]
        else: levels[phi].append([fn, lam])

    for phi,llist in levels.items():
        np = sum( (1 for p in llist if p[1] > 0) )
        nm = sum( (1 for p in llist if p[1] < 0) )
        if np > 0 and nm > 0:
            print "Inconsistent naming for Phi=", phi
            continue
        if nm < 1: continue
        dang = 360.0 / len(llist)
        for v in llist:
            ang = int(-dang*v[1])
            nfn = v[0].replace("_I%03d." % -v[1], "_L%03d." % ang)
            idir = fileman.imageDir + "/"
            os.rename(idir + v[0], idir + nfn)
            print "Renamed: ", nfn
        

def createModel(modelDir, modelName):
    builder = CModelBuilder()
    name = modelName
    amodel = model.CObjectModel(name)
    amodel.FM.setModelStorePath(modelDir)
    renameImageFiles(amodel.FM)
    imdir = amodel.FM.imageDir
    mdldir = amodel.FM.featureDir

    # process views only if processed data doesn't exist
    viewsBuilt = False
    for fn in sorted(os.listdir(mdldir)):
        if fn.startswith(name + ".view.") and fn.endswith(".dat.gz"):
            viewsBuilt = True
            break
    if viewsBuilt:
        print "Loading model"
        amodel.loadViews(mdldir, name)
        print len(amodel.viewPoints), "viewpoints loaded"
    else: builder.buildModelViewpoints(amodel, imdir, mdldir)

    # Assign lambda, phi to views
    # Phis are given in the filename; lambdas must be calculated from index (assume equidistant)
    reId = re.compile("[a-z]+_VP([0-9]+)_L([0-9]+)", re.IGNORECASE)
    lambdaPhi = (2, 1)
    lidx = []
    needsSaving = False
    for i, vp in enumerate(amodel.viewPoints):
        if not isinstance(vp, camview.CViewPoint): vp = amodel.viewPoints[i] # Workaround for late-loading
        mo = reId.search(vp.id)
        if mo != None: lidx.append( [ i, int(mo.group(lambdaPhi[0])), int(mo.group(lambdaPhi[1])) ] )
        if vp.vpLambda == None or vp.vpPhi == None: needsSaving = True
    phis = set([ lf[2] for lf in lidx]); phis = sorted([i for i in phis])

    for phi in phis:
        nv = 1 + max( [ lf[1] for lf in lidx if lf[2] == phi ] ) # max lambda-index for a given phi
        fac = 360.0 / nv
        for lf in lidx:
            if lf[2] == phi: lf[1] = int(lf[1] * fac)

    for lf in lidx:
        i = lf[0]
        amodel.viewPoints[i].vpLambda = lf[1]
        amodel.viewPoints[i].vpPhi = lf[2]
        print lf

    if needsSaving:
        print "Saving"
        amodel.saveViews(mdldir, name)
        needsSaving = False

    # Calculate some view interactins
    if not os.path.exists("%s/%s.model.dat.gz"):
        pass

def compareModelViews(amodel):
    M = matcher.CObjectMatcher()
    comp = comparator.CDescriptorMatcher(nBest = 2)
    for i in xrange(len(amodel.viewPoints)):
        for j in xrange(len(amodel.viewPoints)):
            if j<=i: continue
            v1 = amodel.viewPoints[i]
            v2 = amodel.viewPoints[j]
            print v1.vpLambda, "vs", v2.vpLambda

            fp1 = v1.featurePacks[0]
            fp2 = v2.featurePacks[0]
            dists = comp.matchLists(fp1.descriptors, fp2.descriptors, maxRatio=0.8)
            if len(dists) < 1: continue

            dists.sort(key=lambda x: x[1][0])
            score = M.calcDistScore(dists)
            print "   Score:", score
            cons = M.calcConsistency(dists, fp1, fp2)

def createTest_twEarlGrey():
    builder = CModelBuilder()
    name = "amodel"
    imdir = "../xdata/images/tests/twearlgrey"
    mdldir = imdir # "../xdata/models/" + name
    amodel = model.CObjectModel(name)

    # process views only if processed data doesn't exist
    viewsBuilt = False
    for fn in sorted(os.listdir(mdldir)):
        if fn.startswith(name + ".view.") and fn.endswith(".dat.gz"):
            viewsBuilt = True
            break
    if viewsBuilt:
        print "Loading model"
        amodel.loadViews(mdldir, name)
        print len(amodel.viewPoints), "viewpoints loaded"
    else: builder.buildModelViewpoints(amodel, imdir, mdldir)

if __name__ == "__main__":
    # createModel_house()
    # processTestHouse()
    # createModel_twEarlGrey()
    # createTest_twEarlGrey()
    adir = "xdata/models"
    # createModel(adir, "ShelcoreCube")
    # createModel(adir, "CvetMetaTea")
    # createModel(adir, "SwGreenTea")
    # createModel(adir, "TwEarlGrey")
    createModel(adir, sys.argv[1])

