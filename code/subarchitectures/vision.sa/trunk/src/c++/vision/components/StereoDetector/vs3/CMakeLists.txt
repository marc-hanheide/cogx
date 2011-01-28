PROJECT(SDvs3)

FIND_PACKAGE( OpenCV REQUIRED )

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


ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

target_link_libraries(${PROJECT_NAME} SDL SDLmain)
target_link_libraries(${PROJECT_NAME} rt)
target_link_libraries(${PROJECT_NAME} ${OPENCV_LDFLAGS})
target_link_libraries(${PROJECT_NAME} SDUtils)

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)























