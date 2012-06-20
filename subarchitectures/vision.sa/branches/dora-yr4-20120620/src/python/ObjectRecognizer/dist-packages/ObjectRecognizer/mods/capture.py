#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jan 2009 
#import opencv.cv as cv
#import opencv.highgui as hg
import cv

def copyFrame(frame, copyData=True):
    copy = cv.cvCreateImage(cv.cvSize(frame.width, frame.height), frame.depth, frame.nChannels)
    if copyData: cv.cvCopy(frame, copy)
    return copy

def swapRB(frame):
    copy = cv.cvCreateImage(cv.cvSize(frame.width, frame.height), frame.depth, frame.nChannels)
    # cv.cvConvertImage(frame, copy, cv.CV_CVTIMG_SWAP_RB)
    cv.cvCvtColor(frame, copy, cv.CV_RGB2BGR)
    return copy

class CCapture:
    def __init__(self):
        self.capture = None
        self.convertRbgBgr = False

    def __del__(self):
        self.stop()

    def grabFrame(self, copy=True):
        if self.capture == None: return None
        frame = hg.cvQueryFrame(self.capture)
        if frame == None: return None
        if self.convertRbgBgr: return swapRB(frame)
        if copy: return copyFrame(frame)
        return frame

    def start(self): pass

    def stop(self):
        if self.capture == None: return 
        try: hg.cvReleaseCapture(self.capture)
        except: pass
        self.capture = None

    def isRunning(self):
        return self.capture != None

class CCameraCapture(CCapture):
    def __init__(self, device = 0, size = (640, 480), framerate = 15):
        CCapture.__init__(self)
        self.device = device
        self.width = size[0]
        self.height = size[1]
        self.fps = framerate

    def start(self):
        self.stop()
        cap = hg.cvCreateCameraCapture(self.device)
        self.capture = cap
        return
        if not cap:
            raise "Could not open device #%d" % self.device
        try:
            hg.cvSetCaptureProperty(cap, hg.CV_CAP_PROP_FPS, self.fps)
        except:
            print "Failed to set input framerate"
        try:
            hg.cvSetCaptureProperty(cap, hg.CV_CAP_PROP_FRAME_WIDTH, self.width)
            hg.cvSetCaptureProperty(cap, hg.CV_CAP_PROP_FRAME_HEIGHT, self.height)
        except:
            print "Failed to set input image size"
        self.capture = cap


    def setSize(self, size):
        self.width = size[0]
        self.height = size[1]
        cap = self.capture
        hg.cvSetCaptureProperty(cap, hg.CV_CAP_PROP_FRAME_WIDTH, self.width)
        hg.cvSetCaptureProperty(cap, hg.CV_CAP_PROP_FRAME_HEIGHT, self.height)

    def setFrameRate(self, framerate):
        if framerate == self.fps: return
        if framerate < 1: framerate = 1
        self.fps = framerate
        if self.capture == None: return 
        cap = self.capture
        hg.cvSetCaptureProperty(cap, hg.CV_CAP_PROP_FPS, self.fps)

    def setBrightness(self, brightness):
        brightness = min(max(0.0, brightness), 1.0)
        if self.capture == None: return 
        cap = self.capture
        # print "BRI", hg.cvGetCaptureProperty(cap, hg.CV_CAP_PROP_BRIGHTNESS)
        hg.cvSetCaptureProperty(cap, hg.CV_CAP_PROP_BRIGHTNESS, brightness)

class CFileCapture(CCapture):
    def __init__(self, filename):
        CCapture.__init__(self)
        self.width = 0
        self.height = 0
        self.file = filename

    def start(self):
        self.stop()
        cap = hg.cvCreateFileCapture(self.file)
        if not cap:
            raise "Could not open file '%s'" % self.file
        sefl.width = hg.cvSetCaptureProperty(cap, hg.CV_CAP_PROP_FRAME_WIDTH)
        self.height = hg.cvSetCaptureProperty(cap, hg.CV_CAP_PROP_FRAME_HEIGHT)
        self.capture = cap

class CLoopback1394Capture(CCapture):
    def __init__(self, device = 0, size = (640, 480)):
        CCapture.__init__(self)
        self.device = device

    def start(self):
        # Run an external program to start the device
        # Requires: libdc1394, dc1394_vloopback, vloopback
        # >  sudo modprobe vloopback
        # >  sudo dc1394_vloopback --vloopback /dev/video1 --width 800 --height 600 --palette rgb24 --pipe
        # if --vloopback /dev/video1 is used then the device in __init__ is most likely 2
        cap = hg.cvCreateCameraCapture(self.device)
        self.capture = cap

