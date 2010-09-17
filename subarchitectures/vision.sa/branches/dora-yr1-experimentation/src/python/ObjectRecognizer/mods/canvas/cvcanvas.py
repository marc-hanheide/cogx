#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:
# © Copyright 2009 Marko Mahnič. All Rights Reserved.

import opencv as cv
import opencv.highgui as hg

class CCanvas(object):
    def __init__(self, width, height):
        self.dc = cv.cvCreateImage(cv.cvSize(width, height), cv.IPL_DEPTH_8U, 3)
        self.colors = {
            # BGR values
            "black": cv.cvScalar(0, 0, 0),
            "white": cv.cvScalar(255, 255, 255),
            "blue":  cv.cvScalar(255, 0, 0),
            "green": cv.cvScalar(0, 255, 0),
            "red":   cv.cvScalar(0, 0, 255),
            "magenta": cv.cvScalar(255, 0, 255),
            "yellow": cv.cvScalar(0, 255, 255),
            "cyan":  cv.cvScalar(255, 0, 255)
        }
        self.lineType = 8
        self.lineWidth = 1

    def _color2scalar(self, color):
        if self.colors.has_key( color ): co = self.colors[color]
        else: co = cv.cvScalar(0, 0, 0, 0)
        return co

    def clear(self, color="black"):
        co = self._color2scalar(color)
        cv.cvSet(self.dc, co)
        pass

    def drawImage(self, image, x, y, size=None):
        if size == None: size = (image.width, image.height)
        elif isinstance(size, int):
            mw = max(image.width, image.height)
            scale = 1.0 * size / mw
            size = (int(scale * image.width), int(scale * image.height))
        elif isinstance(size, float):
            size = (int(size * image.width), int(size * image.height))
        elif isinstance(size, tuple): pass
        else: print "Wrong size parameter"; return

        dcsize = (self.dc.width, self.dc.height)
        src = [ 0, 0, size[0], size[1] ]
        dst = [ x, y, x + size[0], y + size[1] ]
        for i in [0, 1]: # fix top-left
            if dst[i] < 0: src[i] += -dst[i]; dst[i] = 0
        for i in [2, 3]: # fix bottom-right
            lim = dcsize[i-2]
            if dst[i] > lim: src[i] -= (dst[i]-lim); dst[i] = lim
        # print src, dst, [ dst[i] - src[i] for i in range(len(src))]
        if dst[2] < 0 or dst[3] < 0: return
        if dst[0] >= self.dc.width or dst[1] >= self.dc.height: return
        # print "Drawing"
        if size[0] != image.width or size[1] != image.height:
            newimg = cv.cvCreateImage(cv.cvSize(size[0], size[1]), cv.IPL_DEPTH_8U, 3)
            cv.cvResize(image, newimg)
            image = newimg

        w = src[2] - src[0]
        h = src[3] - src[1]
        regDst = cv.cvGetSubRect(self.dc, cv.cvRect(dst[0], dst[1], w, h))
        regSrc = cv.cvGetSubRect(image, cv.cvRect(src[0], src[1], w, h))
        cv.cvCopy(regSrc, regDst)

    def drawPoint(self, x, y, shape=".", color="black"):
        co = self._color2scalar(color)
        if shape == "o": cv.cvCircle(self.dc, cv.cvPoint(x, y), 4, co, self.lineWidth, self.lineType)
        elif shape == "O": cv.cvCircle(self.dc, cv.cvPoint(x, y), 8, co, self.lineWidth, self.lineType)
        elif shape == "x":
            sz=3
            cv.cvLine(self.dc, cv.cvPoint(x-sz, y-sz), cv.cvPoint(x+sz, y+sz), co, self.lineWidth, self.lineType)
            cv.cvLine(self.dc, cv.cvPoint(x+sz, y-sz), cv.cvPoint(x-sz, y+sz), co, self.lineWidth, self.lineType)
        elif shape == "X":
            sz=7
            cv.cvLine(self.dc, cv.cvPoint(x-sz, y-sz), cv.cvPoint(x+sz, y+sz), co, self.lineWidth, self.lineType)
            cv.cvLine(self.dc, cv.cvPoint(x+sz, y-sz), cv.cvPoint(x-sz, y+sz), co, self.lineWidth, self.lineType)
        elif shape == "r":
            sz=3
            cv.cvRectangle(self.dc, cv.cvPoint(x-sz, y-sz), cv.cvPoint(x+sz, y+sz), co, self.lineWidth, self.lineType)
        elif shape == "R":
            sz=7
            cv.cvRectangle(self.dc, cv.cvPoint(x-sz, y-sz), cv.cvPoint(x+sz, y+sz), co, self.lineWidth, self.lineType)
        else: cv.cvLine(self.dc, cv.cvPoint(x, y), cv.cvPoint(x+1, y), co, 2)

    def drawLine(self, x0, y0, x1, y1, shape="-", color="black"):
        co = self._color2scalar(color)
        cv.cvLine(self.dc, cv.cvPoint(x0, y0), cv.cvPoint(x1, y1), co, self.lineWidth, self.lineType)


if __name__ == "__main__":
    win = "TEST cvcanvas"
    dc = CCanvas(800, 600)
    dc.clear("blue")

    image = CCanvas(240, 320); image.clear("green") # Just to have an image
    dc.drawImage(image.dc, 20, 20)
    image.clear("yellow")
    png = hg.cvLoadImage("../mods/test/csegmentor00.png")
    image.drawImage(png, 0, 80, size=(240, 180))
    dc.drawImage(image.dc, 400, 300)
    dc.drawImage(image.dc, 400, 300, size=0.5)
    dc.drawImage(image.dc, 500, 500, size=(60, 60))
    if 1:
        print "Clipped"
        dc.drawImage(image.dc, -10, -10)
        dc.drawImage(image.dc, 710, 510)
        print "Outsiders"
        dc.drawImage(image.dc, -1710, -1510)
        dc.drawImage(image.dc, 1710, 1510)

    for i, s in enumerate(".oOxXrR."): dc.drawPoint(15+i*15, 15+i*15, s, color="magenta")
    dc.drawLine(15, 60, 150, 195, color="green")

    hg.cvNamedWindow(win, hg.CV_WINDOW_AUTOSIZE)
    hg.cvShowImage(win, dc.dc)
    while 1:
        k = hg.cvWaitKey(10)
        if k == '\x1b': break



