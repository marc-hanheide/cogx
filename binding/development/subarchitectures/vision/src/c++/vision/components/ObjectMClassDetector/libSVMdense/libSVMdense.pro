include ( ../begin.pri )

TEMPLATE = lib
TARGET = SVMdense

# Do not link againt QT libraries
QT -= core gui

DEFINES += DENSE

# Input
HEADERS += ../../include/libSVMdense/kernel.h \
			../../include/libSVMdense/svm_classify.h \
			../../include/libSVMdense/svm_common.h \
			../../include/libSVMdense/svm_learn.h

SOURCES += binary_format.cpp \
           svm_common.cpp \
           svm_hideo.cpp \
           svm_learn.cpp \          
           svm.cpp

 include(../end.pri)
