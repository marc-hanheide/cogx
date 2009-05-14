
include( ../begin.pri )

TEMPLATE = lib
TARGET = Annotation
VERSION = 2.0

## Do not link againt QT libraries
QT -= core gui

# Input
SOURCES += xmlhelpers.cpp annorect.cpp annotation.cpp annotationlist.cpp

include( ../end.pri )

