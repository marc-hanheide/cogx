TEMPLATE = lib

CONFIG += staticlib

CONFIG -= qt \
thread
SOURCES += alloc.cpp \
choleski.cpp \
construc.cpp \
creat.cpp \
inverse.cpp \
io.cpp \
is.cpp \
jacobi.cpp \
jordan.cpp \
lu.cpp \
manip.cpp \
norm.cpp \
operator.cpp \
points.cpp \
ql.cpp \
qr.cpp \
quatern.cpp \
rotation.cpp \
svdcmp.cpp \
system.cpp \
unary.cpp \
vector.cpp
HEADERS += dmatrix.h \
matrix.h \
matrix.hh \
points.hh \
quatern.hh \
vector.hh
DISTFILES += matrix.icc \
vector.icc
