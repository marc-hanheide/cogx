CONFIG -= qt

CONFIG += debug_and_release \
staticlib
TEMPLATE = lib

HEADERS += mxDrawTools.h \
 mxCvTools.h \
 mxCameraModel.h

SOURCES += mxDrawTools.cpp \
 mxCameraModel.cpp


LIBS += -lcv

