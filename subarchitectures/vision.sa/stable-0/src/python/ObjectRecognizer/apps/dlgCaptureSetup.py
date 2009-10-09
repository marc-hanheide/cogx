#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: Aug 2009 

import os, sys, time
import os.path
from PyQt4 import QtCore, QtGui

from qtui import uiCaptureSetup

class CCaptureParams:
    def __init__(self, lambdaStep=45, phiList=[0, 15, 30, 45, 90]):
        self.lambdaStep = lambdaStep
        self.lambdaReverse = False
        self.phiList = phiList

class CCaptureSetupDlg(QtGui.QDialog):
    def __init__(self):
        QtGui.QDialog.__init__(self)
        self.ui = uiCaptureSetup.Ui_dlgCaptureSetup()
        self.ui.setupUi(self)
        self.params = None
        self._initialized = False

    def showEvent(self, event):
        if not self._initialized:
            self.initControls()
            self._initialized = True

    def useParams(self, paramCapture):
        self._initialized = False
        self.params = paramCapture
    
    def initControls(self):
        if self.params == None: return
        phis = ", ".join( [ "%d" % phi for phi in self.params.phiList ] )
        self.ui.txtPhiList.setText(phis)
        self.ui.txtLambdaStep.setText("%d" % self.params.lambdaStep)
        self.ui.ckReverse.setChecked(self.params.lambdaReverse)

    def parseControls(self):
        errors = []
        if self.params == None: return errors
        try:
            lmbd = int("%s" % self.ui.txtLambdaStep.text())
            self.params.lambdaStep = lmbd
        except:
            errors.append("Invalid value for Lambda step")

        try:
            philist = [0]
            phis = "%s" % self.ui.txtPhiList.text()
            for p in phis.split(","):
                try: philist.append(int(p.strip()))
                except: errors.append("Invalid value for Phi (%s)" % p)
            self.params.phiList = sorted(set(philist))
        except: errors.append("Error parsing Phi List")

        self.params.lambdaReverse = self.ui.ckReverse.isChecked()

        return errors

    def accept(self):
        self.parseControls()
        self.done(1)

    def reject(self):
        self.done(0)




