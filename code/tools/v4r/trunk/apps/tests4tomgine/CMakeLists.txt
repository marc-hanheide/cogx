

SET(APPS_TESTS4TOMGINE 0 CACHE BOOL "Add description here." )
IF(APPS_TESTS4TOMGINE)

	find_package(PCL 1.0 REQUIRED)
	include_directories(${PCL_INCLUDE_DIRS})
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})
  
	set(OPENNURBS_PATH /usr/local/lib/opennurbs/ CACHE PATH "Path to OpenNURBS library (www.opennurbs.org).")
	
	include_directories(${OPENNURBS_PATH})
	link_directories(${OPENNURBS_PATH})
	set(TESTS4TOMGINE_SRC
		main_tests4tomgine.cpp
		NurbsTools.cpp
		DataLoading.cpp
		PatchFitting.cpp
	)
	
	add_executable(tests4tomgine ${TESTS4TOMGINE_SRC})
	target_link_libraries(tests4tomgine v4rtomgine v4rglwindow)
	target_link_libraries(tests4tomgine openNURBS)

ENDIF(APPS_TESTS4TOMGINE)
