SET(APPS_TESTSTRACKER 0 CACHE BOOL "Add description here." )
IF(APPS_TESTSTRACKER)

	set(SOURCE_CPP 
		main_teststracker.cpp)

	add_executable(trackertest ${SOURCE_CPP})
	target_link_libraries(trackertest v4rtracker)

ENDIF(APPS_TESTSTRACKER)
