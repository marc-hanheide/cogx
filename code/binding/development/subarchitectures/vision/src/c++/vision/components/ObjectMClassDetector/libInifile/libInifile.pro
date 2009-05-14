

include ( ../begin.pri )

TEMPLATE = lib
TARGET = Inifile
VERSION = 1.0

SOURCES += inifile.cpp

QT -= core gui
                    
HEADERS += $$system(ls $${BASE}/include/lib$${TARGET}/*.h)
include(../end.pri)

