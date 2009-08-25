TEMPLATE = app



TARGET = vs3

DESTDIR = ../bin/

SOURCES += Arc.cc \
betacf.c \
betai.c \
BufferVideo.cc \
Closure.cc \
Collinearity.cc \
Color.cc \
Cone.cc \
Config.cc \
ConfigFile.cc \
ConvexArcGroup.cc \
Corner.cc \
Cube.cc \
Cylinder.cc \
Draw.cc \
EJunction.cc \
Ellipse.cc \
Except.cc \
Exit.cc \
ExtClosure.cc \
ExtEllipse.cc \
ExtRectangle.cc \
FileVideo.cc \
FireVideo.cc \
fitcircle.c \
fitellipse.c \
Flap.cc \
FormArcs.cc \
FormClosures.cc \
FormCones.cc \
FormConvexArcGroups.cc \
FormCorners.cc \
FormCubes.cc \
FormCylinders.cc \
FormEllipses.cc \
FormExits.cc \
FormExtClosures.cc \
FormExtEllipses.cc \
FormExtRectangles.cc \
FormFlaps.cc \
FormJunctions.cc \
FormLines.cc \
FormParallelLineGroups.cc \
FormRectangles.cc \
FormSegments.cc \
FormWallLines.cc \
FormWalls.cc \
gammln.c \
Gestalt.cc \
GestaltPrinciple.cc \
HCF2.cc \
IceInterface.cc \
IdImage.cc \
Image.cc \
Line.cc \
LJunction.cc \
Main.cc \
MainWin.cc \
Math.cc \
nrutil.c \
Object.cc \
ObjectTracker.cc \
OpenCvVideo.cc \
ParallelLineGroup.cc \
Rectangle.cc \
rosin_arcline.cc \
rosin_lines.cc \
Segment.cc \
TJunction.cc \
TObject.cc \
Vector2.cc \
VisionCore.cc \
VoteImage.cc \
Wall.cc \
WallLine.cc \
 BestResult.cc \
 FormBestResults.cc \
 Ball.cc \
 FormBalls.cc \
 FormMotionField.cc \
 MotionFieldElement.cc \
 TrktRectangle.cc \
 TrackRectangles.cc \
 MotionField.cc \
 TrackFlaps.cc \
 TrktFlap.cc \
 TrktCube.cc \
 TrackCubes.cc
HEADERS += Arc.hh \
Array.hh \
BufferVideo.hh \
Closure.hh \
Collinearity.hh \
Color.hh \
ConeDefinition.hh \
Cone.hh \
ConfigFile.hh \
Config.hh \
ConvexArcGroup.hh \
Corner.hh \
CubeDefinition.hh \
Cube.hh \
CylinderDefinition.hh \
Cylinder.hh \
Draw.hh \
Edgel.hh \
EJunction.hh \
Ellipse.hh \
Except.hh \
ExitDefinition.hh \
Exit.hh \
ExtClosure.hh \
ExtEllipse.hh \
ExtRectangle.hh \
FileVideo.hh \
FireVideo.hh \
fitcircle.h \
fitellipse.h \
Flap.hh \
FormArcs.hh \
FormClosures.hh \
FormCones.hh \
FormConvexArcGroups.hh \
FormCorners.hh \
FormCubes.hh \
FormCylinders.hh \
FormEllipses.hh \
FormExits.hh \
FormExtClosures.hh \
FormExtEllipses.hh \
FormExtRectangles.hh \
FormFlaps.hh \
FormJunctions.hh \
FormLines.hh \
FormParallelLineGroups.hh \
FormRectangles.hh \
FormSegments.hh \
FormWallLines.hh \
FormWalls.hh \
Gestalt.hh \
GestaltPrinciple.hh \
HCF.hh \
IceInterface.hh \
IdImage.hh \
Image.hh \
Line.hh \
LJunction.hh \
MainWin.hh \
Math.hh \
Namespace.hh \
nrutil.h \
Object.hh \
ObjectTracker.hh \
OpenCvVideo.hh \
ParallelLineGroup.hh \
Rectangle.hh \
rosin_arcline.hh \
rosin_lines.hh \
Segment.hh \
TJunction.hh \
TObject.hh \
Vector2.hh \
Video.hh \
VisionCore.hh \
VoteImage.hh \
WallDefinition.hh \
Wall.hh \
WallLine.hh \
 BestResult.hh \
 FormBestResults.hh \
 Ball.hh \
 FormBalls.hh \
 BallDefinition.hh \
 FormMotionField.hh \
 MotionFieldElement.hh \
 LJunctionDefinition.hh \
 TrktRectangle.hh \
 TrackRectangles.hh \
 MotionField.hh \
 RectangleDefinition.hh \
 TrackFlaps.hh \
 TrktFlap.hh \
 TrktCube.hh \
 TrackCubes.hh \
 TCubeDefinition.hh \
 TFlapDefinition.hh
DISTFILES += Array.ic \
Camera.ic \
Math.ic \
Vector2.ic

CONFIG += debug_and_release












QMAKE_CXXFLAGS_RELEASE += -DHAVE_IMLIB \
  -DUSE_OPENCV_ELLFIT \
  -DUSE_COLOR \
  -DGL_GLEXT_PROTOTYPES

QMAKE_CXXFLAGS_DEBUG += -DHAVE_IMLIB \
  -DUSE_OPENCV_ELLFIT \
  -DUSE_COLOR \
  -DGL_GLEXT_PROTOTYPES







INCLUDEPATH += ../ice_vision/slice_message \
  ../mxCvTools \
  ../matas \
  ../Tracker3D

TARGETDEPS += ../ice_vision/slice_message/libslice_message.a \
  ../mxCvTools/libmxCvTools.a \
  ../matas/libmatas.a
LIBS += ../mxCvTools/libmxCvTools.a \
  ../ice_vision/slice_message/libslice_message.a \
  ../matas/libmatas.a \
  ../Tracker3D/libTracker3D.a \
  -L/usr/X11/lib \
  -lSDL \
  -lSDLmain \
  -lImlib \
  -lglut \
  -lGLU \
  -lIce \
  -lIceUtil \
  -lrt \
  -lm \
  -ldl \
  -lpthread \
  -lhighgui \
  -lcxcore \
  -lcv \
  -lqt-mt

