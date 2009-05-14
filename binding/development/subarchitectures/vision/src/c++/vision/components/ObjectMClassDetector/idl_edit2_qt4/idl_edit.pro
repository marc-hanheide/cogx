######################################################################

CONFIG += qt release

TEMPLATE = app
TARGET = ../idleditor
ARCH = i686

# Input
HEADERS += idl_edit.hh \
           idl_editimgdisp.hh \
           idl_parser.hh \
           imgdescr.hh \
           imgdescrlist.hh \
           qtcoordlabel.hh \
           qtresizeimg.hh
SOURCES += idl_edit.cc \
           idl_editimgdisp.cc \
           idl_parser.cc \
           imgdescrlist.cc \
           main.cc \
           qtcoordlabel.cc \
           qtresizeimg.cc

# dirs

#The following line was inserted by qt3to4
QT +=  qt3support 
