#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jan 2009 

from PyQt4 import QtCore, QtGui
import opencv.cv as cv
import opencv.highgui as hg
import opencv.adaptors as cvada
import numpy as np

def qtImageFromIpl(image):
    cvimg = cvada.Ipl2NumPy(image) # TODO: slow, improve!
    npimg = np.ndarray(shape=(image.rows, image.cols, 4), dtype = np.byte)
    npimg[:, :, :3] = cvimg[:, :, :3]
    # imqt will use the memory allocated by npimg;
    # the image has to be copied to duplicate the memory, otherwise it will segfault later.
    imqt = QtGui.QImage(npimg, image.cols, image.rows, QtGui.QImage.Format_RGB32)
    return imqt.copy()

def qtImageThumbFromIpl(image, height=120):
    cvimg = cvada.Ipl2NumPy(image) # TODO: slow, improve!
    npimg = np.ndarray(shape=(image.rows, image.cols, 4), dtype = np.byte)
    npimg[:, :, :3] = cvimg[:, :, :3]
    # imqt will use the memory allocated by npimg;
    imqt = QtGui.QImage(npimg, image.cols, image.rows, QtGui.QImage.Format_RGB32)
    thumb = imqt.scaledToHeight(height)
    #pix = QtGui.QPixmap(thumb.width(), thumb.height())
    #PT = QtGui.QPainter(pix)
    #PT.drawImage(0, 0, thumb)
    #PT.end()
    #return pix
    return thumb

def cvImageFromQt(qtImage):
    bits = qtImage.bits()
    pass
