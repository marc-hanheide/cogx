SOURCES += arg2Str.c \
ary.c \
buffStr.c \
canny_body.c \
colorquant.c \
consP.c \
consStr.c \
ConstSet.c \
consVectsP.c \
convertAry.c \
copyAry.c \
destSets.c \
dupStr.c \
entityP.c \
errorHand.c \
extendAry.c \
formatP.c \
getSetP.c \
gradient.c \
graph.c \
initAry.c \
LL.c \
LLconsis.c \
LLfile.c \
LLio.c \
LLmergeSort.c \
LLstr.c \
makary.c \
normFary.c \
option.c \
optionDouble.c \
optionInt.c \
optionList.c \
optionMisc.c \
PGerror.c \
PGio.c \
PGutil.c \
quantAry.c \
readAry.c \
RWSets.c \
scaleFary.c \
StdGeomDefP.c \
strcasecmp.c \
suffStr.c \
thrLink.c \
tokenPref.c \
tokenStr.c \
vector2.c \
vector2i.c \
writeAry.c \
writeGF.c \
xyEdge2LL.c
HEADERS += ary.h \
aryio.h \
cannyGradGlob.h \
canny.h \
colorquant.h \
gfLLconv.h \
gfLL.h \
gfLLio.h \
gfPars.h \
graph.h \
linkLL.h \
LL.h \
option.h \
optionPriv.h \
strGM.h \
vector2.h \
 MotionDefinition.hh
TEMPLATE = lib

CONFIG += staticlib \
debug_and_release
CONFIG -= qt

