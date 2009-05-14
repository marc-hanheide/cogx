include ( ../begin.pri )

TEMPLATE = lib
TARGET = HOG

## Do not link againt QT libraries
#QT-=gui

#CONFIG+=release
#CONFIG-=debug
#CONFIG-=release
#CONFIG+=debug

SOURCES += HOGparams.cpp hog.cpp HOGTest.cpp HOGModel.cpp hogConf.cpp
#hogwrapper.cpp hogConf.cpp
                      
HEADERS += $$system(ls $${BASE}/include/lib$${TARGET}/*.h)
include(../end.pri)

