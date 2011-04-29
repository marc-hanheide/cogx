PROJECT(SFSrc)

include_directories(.)

# Set the headers.
SET(HEADERS
	fitcircle.h
	fitellipse.h
	nrutil.h
	Arc.hh
	Array.hh
	Ball.hh
	BallDefinition.hh
	BestResult.hh
	BufferVideo.hh
	Closure.hh
	Collinearity.hh
	Color.hh
	Cone.hh
	ConeDefinition.hh
	Config.hh
	ConfigFile.hh
	ConvexArcGroup.hh
	Corner.hh
	Cube.hh
	CubeDefninition.hh
	Draw.hh
	Edgel.hh
	EJunction.hh
	Ellipse.hh
	Exept.hh
	Exit.hh
	ExitDefinition.hh
	ExtClosure.hh
	ExtEllipse.hh
	ExtRectangle.hh
	FileVideo.hh
	FireVideo.hh
	Flap.hh
	FlapDefinition.hh
	FormArcs.hh
	FormBalls.hh
	FormBestResults.hh
	FormClosures.hh
	FormCones.hh
	FormConvexArcGroups.hh
	FormCorners.hh
	FormCubes.hh
	FormCylinders.hh
	FormEllipses.hh
	FormExits.hh
	FormExtClosures.hh
	FormExtEllipses.hh
	FormExtRectangles.hh
	FormFlaps.hh
	FormJunctions.hh
	FormLines.hh
	FormMotionField.hh
#	FormParallelLineGroups.hh
#	FormRectangles.hh
	FormSegments.hh
	FormWallLines.hh
	FormWalls.hh
	Gestalt.hh
	GestaltPrinciple.hh
	HCF.hh
#	IceInterface.hh
	IdImage.hh
	Image.hh
	Line.hh
#	LiveVideo.hh
	LJunction.hh
	LJunctionDefinition.hh
#	MainWin.hh
	Math.hh
	MotionDefinition.hh
	MotionField.hh
	MotionFieldElement.hh
	Namespace.hh
	Object.hh
	ObjectTracker.hh
	OpenCvVideo.hh
	ParallelLineGroup.hh
	Rectangle.hh
	RectangleDefinition.hh
	rosin_arcline.hh
	rosin_lines.hh
	Segment.hh
	TCubeDefinition.hh
	TFlapDefinition.hh
	TJunction.hh
	TObject.hh
	TrackCubes.hh
	TrackFlaps.hh
	TrackRectangles.hh
	TrktCube.hh
	TrktFlap.hh
	TrktRectangle.hh
	Vector2.hh
	Video.hh
	VisionCore.hh
	VoteImage.hh
	Wall.hh
	WallDefinition.hh
	WallLine.hh)

SET(SOURCES 
	Array.ic
	betacf.c
	betai.c
	Camera.ic
	fitcircle.c
	fitellipse.c
	gammln.c
	Math.ic
	nrutil.c
	Vector2.ic
	AJunction.cc
	Arc.cc
#	Ball.cc
#	BestResult.cc
#	BufferVideo.cc
	Closure.cc
	Collinearity.cc
	Color.cc
#	Cone.cc
	Config.cc
	ConfigFile.cc
	ConvexArcGroup.cc
#	Corner.cc
#	Cube.cc
#	Cylinder.cc

	Draw.cc

#	Draw_small.cc
#	EJunction.cc
	Ellipse.cc
	Except.cc
#	Exit.cc
#	ExtClosure.cc
#	ExtEllipse.cc
#	ExtRectangle.cc
#	FileVideo.cc
#	FireVideo.cc
	Flap.cc
	FormArcJunctions.cc
	FormArcs.cc
#	FormBalls.cc
#	FormBestResults.cc
	FormClosures.cc
#	FormCones.cc
	FormConvexArcGroups.cc
	FormEllipses.cc
#	FormCorners.cc
#	FormCubes.cc
#	FormCylinders.cc
#	FormEllipses.cc
#	FormExits.cc
#	FormExtClosures.cc
#	FormExtEllipses.cc
#	FormExtRectangles.cc
	FormFlaps.cc
	FormJunctions.cc
	FormLines.cc
#	FormMotionField.cc
#	FormParallelLineGroups.cc
#	FormRectangles.cc
	FormSegments.cc
#	FormWallLines.cc
#	FormWalls.cc
	Gestalt.cc
	GestaltPrinciple.cc
#	HCF2.cc
#	IceInterface.cc
	IdImage.cc
	Image.cc
	Line.cc
#	LiveVideo.cc
	LJunction.cc
#	Main.cc
#	main_small.cc
 #	MainWin.cc
	Math.cc
 #	moc_MainWin.cpp
#	MotionField.cc
#	MotionFieldElement.cc
#	Object.cc
#	ObjectTracker.cc
#	OpenCvBufferVideo.cc
	OpenCvImgSeqVideo.cc
	OpenCvLiveVideo.cc
#	OpenCvVideo.cc
#	ParallelLineGroup.cc
	Rectangle.cc
	rosin_arcline.cc
#	rosin_arcs.cc			=> in FormArcs.cc direkt eingebunden!
	rosin_lines.cc
	Segment.cc
	TJunction.cc
#	TObject.cc
#	TrackCubes.cc
#	TrackFlaps.cc
#	TrackRectangles.cc
#	TrktCube.cc
#	TrktFlap.cc
#	TrktRectangle.cc
	Vector2.cc
	VisionCore.cc
	VoteImage.cc
#	Wall.cc
#	WallLine.cc
)


# INCLUDE DIRECTORIES WITH HEADERS (INTERN)
#include_directories()


# INCLUDE DIRECTORIES WITH HEADERS (EXTERN)
include_directories(/usr/local/include)
include_directories(/usr/include)
include_directories(/usr/include/qt3)


link_cast_component(${CAST_COMPONENT_NAME} glut)
link_cast_component(${CAST_COMPONENT_NAME} GLU)
link_cast_component(${CAST_COMPONENT_NAME} SDL)
link_cast_component(${CAST_COMPONENT_NAME} SDLmain)
link_cast_component(${CAST_COMPONENT_NAME} Imlib)
link_cast_component(${CAST_COMPONENT_NAME} rt)
link_cast_component(${CAST_COMPONENT_NAME} m)
link_cast_component(${CAST_COMPONENT_NAME} dl)
link_cast_component(${CAST_COMPONENT_NAME} pthread)
link_cast_component(${CAST_COMPONENT_NAME} qt-mt)
link_cast_component(${CAST_COMPONENT_NAME} ${OPENCV_LDFLAGS})


#link_cast_component(${CAST_COMPONENT_NAME} matas)
#link_cast_component(${CAST_COMPONENT_NAME} mxCvTools)
#link_cast_component(${CAST_COMPONENT_NAME} Vs3Matas Vs3MxCvTools)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)























