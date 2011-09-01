SET(APPS_NURBSFITTING 0 CACHE BOOL "Add description here." )
IF(APPS_NURBSFITTING)

	find_package(PCL 1.0 REQUIRED)
	include_directories(${PCL_INCLUDE_DIRS})
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})
	
	set(OPENNURBS_PATH /usr/local/lib/opennurbs/ CACHE PATH "Path to OpenNURBS library (www.opennurbs.org).")
	include_directories(${OPENNURBS_PATH})
	link_directories(${OPENNURBS_PATH})

	set(OPENMESH_LIBS /usr/local/lib/OpenMesh CACHE PATH "Path to OpenMesh library (www.openmesh.org).")
	link_directories(${OPENMESH_LIBS})	
	
	set(EXTERNALS ${V4R_DIR}/external)
	include_directories(${EXTERNALS})
	link_directories(${EXTERNALS}/lib)
	
	set(SOURCE_CPP
		PopoutProcessor.cpp
		NurbsCreator.cpp
		NurbsTools.cpp
		main.cpp
	)

	add_executable(nurbsfitting ${SOURCE_H} ${SOURCE_CPP})
	target_link_libraries(nurbsfitting v4rtomgine v4rglwindow)
	target_link_libraries(nurbsfitting PlanePopout ${PCL_LIBRARIES} PCLUtils)
	target_link_libraries(nurbsfitting igal openNURBS lpsolve55)
	target_link_libraries(nurbsfitting blztools OpenMeshCore lapack cholmod umfpack armadillo ann)

ENDIF(APPS_NURBSFITTING)
