include ( ../begin.pri )

TEMPLATE = lib
TARGET = ChiSquare
VERSION = 3.0

#Do not link againt QT libraries
QT -= core gui

SOURCES += chisquare.cc gamma.cc

HEADERS += $$system(ls $${BASE}/include/lib$${TARGET}/*.h*)
include( ../end.pri )

