include ( ../begin.pri )

TEMPLATE = lib
TARGET = PPProcess

## Do not link againt QT libraries
#QT-=gui

#CONFIG+=release
#CONFIG-=debug

SOURCES += nonmaxsuppression.cpp preprocess.cpp sharpen.cpp
                      
HEADERS += $$system(ls $${BASE}/include/lib$${TARGET}/*.h)
include(../end.pri)

