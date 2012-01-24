PROJECT(SDvs3)

SET(SOURCES 
	betacf.c
	betai.c
	gammln.c
	nrutil.c
	AJunction.cc
	Arc.cc
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
	FlapAri.cc
	Flap.cc
	FormArcJunctions.cc
	FormArcs.cc
	FormCircles.cc
	FormClosures.cc
	FormCones.cc
	FormConvexArcGroups.cc
	FormCorners.cc
	FormCubes.cc
	FormEllipses.cc
	FormEJunctions.cc
	FormCylinders.cc
	FormFlapsAri.cc
	FormFlaps.cc
	FormJunctions.cc
	FormLines.cc
	FormRectangles.cc
	FormSegments.cc
	Gestalt.cc
	GestaltPrinciple.cc
 	IdImage.cc
	Line.cc
	LJunction.cc
	Rectangle.cc
	rosin_arcline.cc
	rosin_lines.cc
	Segment.cc
	TJunction.cc
	VisionCore.cc
	VoteImage.cc
)

# INCLUDE DIRECTORIES WITH HEADERS (INTERN)
include_directories(.)

ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})

target_link_libraries(${PROJECT_NAME} rt)
target_link_libraries(${PROJECT_NAME} SDUtils SDMatas)

INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)























