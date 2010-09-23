PROJECT(SDvs3)

include_directories(.)

SET(SOURCES 
	betacf.c
	betai.c
#	fitcircle.c
#	fitellipse.c
	gammln.c
	Math.ic
	nrutil.c
#	Vector2.ic
	AJunction.cc
	Arc.cc
#	BestResult.cc
#	Camera.cc
	CEdge.cc
	Circle.cc
	Closure.cc
	Collinearity.cc
	Cone.cc
	ConvexArcGroup.cc
	Corner.cc
	Cube.cc
	Cylinder.cc
	EJunction.cc
	Ellipse.cc
#	Exit.cc
#	ExtClosure.cc
#	ExtEllipse.cc
#	ExtRectangle.cc
#	FileVideo.cc
#	FireVideo.cc
	FlapAri.cc
	Flap.cc
	FormArcJunctions.cc
	FormArcs.cc
#	FormBestResults.cc
	FormCircles.cc
	FormClosures.cc
	FormCones.cc
	FormConvexArcGroups.cc
	FormCorners.cc
	FormCubes.cc
	FormEllipses.cc
	FormEJunctions.cc
	FormCylinders.cc
#	FormEllipses.cc
#	FormExits.cc
#	FormExtClosures.cc
#	FormExtEllipses.cc
#	FormExtRectangles.cc
	FormFlapsAri.cc
	FormFlaps.cc
	FormJunctions.cc
	FormLines.cc
#	FormMotionField.cc
#	FormParallelLineGroups.cc
	FormRectangles.cc
	FormSegments.cc
#	FormWallLines.cc
#	FormWalls.cc
	Gestalt.cc
	GestaltPrinciple.cc
#	HCF2.cc
#	IceInterface.cc
 	IdImage.cc
#	Image.cc
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
#	OpenCvImgSeqVideo.cc
#	OpenCvLiveVideo.cc
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
#	Vector2.cc
	VisionCore.cc
	VoteImage.cc
#	Wall.cc
#	WallLine.cc
)


# INCLUDE DIRECTORIES WITH HEADERS (INTERN)
include_directories(.)


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
link_cast_component(${CAST_COMPONENT_NAME} highgui)
link_cast_component(${CAST_COMPONENT_NAME} cxcore)
link_cast_component(${CAST_COMPONENT_NAME} cv)
link_cast_component(${CAST_COMPONENT_NAME} qt-mt)

link_cast_component(${CAST_COMPONENT_NAME} SDUtils)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)























