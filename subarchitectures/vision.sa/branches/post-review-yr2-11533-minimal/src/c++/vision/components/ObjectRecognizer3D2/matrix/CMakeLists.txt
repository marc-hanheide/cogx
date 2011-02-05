
project(matrix2)

include_directories(.)

SET(SOURCES 
		alloc.cpp
		choleski.cpp
		construc.cpp
		creat.cpp
		inverse.cpp
		io.cpp
		is.cpp
		jacobi.cpp
		jordan.cpp
		lu.cpp
		manip.cpp
		norm.cpp
		operator.cpp
		points.cpp
		ql.cpp
		qr.cpp
		quatern.cpp
		rotation.cpp
		svdcmp.cpp
		system.cpp
		unary.cpp
		vector.cpp)
		

add_library(${PROJECT_NAME} SHARED ${SOURCES})
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
