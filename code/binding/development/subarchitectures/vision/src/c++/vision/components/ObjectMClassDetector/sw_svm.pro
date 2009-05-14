
TEMPLATE = app
TARGET = sw_svm

#CONFIG -= release
#CONFIG += debug

## Do not link againt QT libraries
#QT-=core gui

QMAKE_CXXFLAGS += -O3
DEFINES += DENSE
SOURCES += main.cpp parameters.cpp train.cpp test.cpp hog_features.cpp \ 
            libAnnotation/annorect.cpp libAnnotation/annotation.cpp libAnnotation/annotationlist.cpp libAnnotation/xmlhelpers.cpp \
            libChiSquare/chisquare.cc libChiSquare/gamma.cc \
            libCmd/BoolOption.C libCmd/CmdOption.C libCmd/CmdParser.C libCmd/EnumOption.C libCmd/StringArgument.C libCmd/StringOption.C \
            libFeatures/featurevector.cc \
            libHOG/HOG.cpp libHOG/HOGparams.cpp \
            libInifile/inifile.cpp \
            libPPProcess/nonmaxsuppression.cpp libPPProcess/preprocess.cpp libPPProcess/sharpen.cpp \
            libSVMdense/binary_format.cpp libSVMdense/svm_common.cpp libSVMdense/svm_hideo.cpp libSVMdense/svm_learn.cpp libSVMdense/svm.cpp
#train.cpp test.cpp

LIBS += -lcv -lhighgui 


