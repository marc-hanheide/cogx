#! /usr/bin/env python
# vim: set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: april 2009 
#import opencv as cv
import cv
import numpy as np
import capture
import time

SHOW_TIMES=False

class CSegmentor:
    def __init__(self):
        self.cellsize = (8, 8)
        pass

    def getObjectRegion1(self, image, center=None, initsize=(1, 1)):
        # image into 8x8 cells
        # histogram for each cell
        # compare to histograms at center
        if center == None:
            center = (image.width / 2, image.height / 2)

    def _getMaskFromBorder(self, image):
        # assume the border doesn't contain the object
        # assume uniform background
        # pixel belongs to bg if similar to neighbours
        newsz = cv.cvSize(image.width / self.cellsize[0], image.height / self.cellsize[1])
        scaled = cv.cvCreateImage(newsz, image.depth, image.nChannels)
        cv.cvResize(image, scaled)
        mask = cv.cvCreateImage(newsz, cv.IPL_DEPTH_8U, 1)
        rflt = 2
        tic = time.time()
        for x in xrange(newsz.width):
            for y in xrange(newsz.height):
                if x < rflt or y < rflt: mask[y, x] = 0
                elif x >= newsz.width - rflt or y >= newsz.height - rflt: mask[y, x] = 0
                else: mask[y, x] = 255
        if SHOW_TIMES: print "_getMaskFromBorder, clear border in", time.time()-tic

        def near(co1, co2):
            dc = np.array([co2[0]-co1[0], co2[1]-co1[1], co2[2]-co1[2], co2[3]-co1[3]])
            dist = np.dot(dc, dc)
            return dist < 60

        cells = []
        for x in xrange(rflt * 2 + 1):
            for y in xrange(rflt * 2 + 1):
                if x != rflt or y != rflt: cells.append( (x-rflt, y-rflt) )
        changes = 1; passes = 0
        while changes:
            changes = 0; passes += 1
            for x in xrange(newsz.width):
                for y in xrange(newsz.height):
                    if mask[y, x] == 0: continue
                    count = 0; color = scaled[y, x]
                    for c in cells:
                        a, b = x + c[0], y + c[1]
                        if mask[b, a] == 0 and near(color, scaled[b, a]): count += 1
                    if count >= 6:
                        mask[y, x] = 0
                        changes += 1
        tmp = capture.copyFrame(mask, copyData=False)
        cv.cvErode(mask, tmp, None, 2)
        cv.cvDilate(tmp, mask, None, 2)
        if SHOW_TIMES: print "_getMaskFromBorder passes =", passes
        return mask

    def getObjectRegion(self, image):
        tic = time.time()
        m1 = self._getMaskFromBorder(image)
        if SHOW_TIMES: print "getObjectRegion in", time.time()-tic
        return m1
        # Flipping doesn't help - still not well placed
        #flipped = capture.copyFrame(image, copyData = False)
        #cv.cvFlip(image, flipped, -1) # -1 = both axes
        #m2f = self._getMaskFromBorder(flipped)
        #m2 = capture.copyFrame(m2f, copyData = False)
        #cv.cvFlip(m2f, m2, -1) # -1 = both axes
        #return m2

if __name__ == "__main__":
    import opencv.highgui as hg
    for i in xrange(3):
        print i
        image = hg.cvLoadImage("test/csegmentor%02d.png" % i)
        S = CSegmentor()
        m = S.getObjectRegion(image)
        if m != None: hg.cvSaveImage("test/csegmentor%02d.mask.png" % i, m)

