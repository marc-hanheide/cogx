# File: vislearner.inc
# vim: ft=conf sw=3

VARDEFAULT visl_dsPrefix = VisLearner

VARDEFAULT visl_displayDir = subarchitectures/vision.sa/config/test-vislearner/files/disp

VARDEFAULT visl_monitorParams = <multiline>
   %(visl_dsPrefix).bmp.Recognition=%(visl_displayDir)/{mGR.png}
   %(visl_dsPrefix).bmp.Learning=%(visl_displayDir)/{mGL.png}
   %(visl_dsPrefix).bmp.D=%(visl_displayDir)/{mGD.png}
   %(visl_dsPrefix).htm.Recognition=%(visl_displayDir)/{mTR.html}
   %(visl_dsPrefix).htm.Learning=%(visl_displayDir)/{mTL.html}
   %(visl_dsPrefix).htm.D=%(visl_displayDir)/{mTD.html}
</multiline>

VARDEFAULT visl_clfConfig = subarchitectures/vision.sa/config/test-vislearner/cmLearn.config

CPP MG vis.learner  VisualLearner --clfconfig "%(visl_clfConfig)" --log # --debug
CPP MG visl.learner.display FileMonitorDC --displayserver "display.srv" --monitor "%(visl_monitorParams)" # --debug true
