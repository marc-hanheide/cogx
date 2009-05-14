
include ( ../begin.pri )

TEMPLATE = lib
TARGET = Features
VERSION = 3.0

#Do not link againt QT libraries
QT -= core gui

SOURCES += featurevector.cc

HEADERS += $$system(ls $${BASE}/include/lib$${TARGET}/*.h*)
include(../end.pri)

