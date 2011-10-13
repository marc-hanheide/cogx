SET(APPS_TESTS4RECOGNISER 0 CACHE BOOL "Add description here." )
IF(APPS_TESTS4RECOGNISER)

  find_package(PCL 1 REQUIRED)

  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  set(SOURCE_H
    TgModelProbSphere.hh
  )

	set(SOURCE_CPP
    TgModelProbSphere.cc
	)

	add_executable(learnRecognise ${SOURCE_H} ${SOURCE_CPP} main.cpp)
	target_link_libraries(learnRecognise v4rtomgine v4rglwindow v4rtracker)
	target_link_libraries(learnRecognise CModelRecogniser v4rpclutils2 v4rpclfunctions2 v4rplanepopout2 EllCalib ${PCL_LIBRARIES})

	add_executable(evalRecognise ${SOURCE_H} ${SOURCE_CPP} eval.cpp)
	target_link_libraries(evalRecognise v4rtomgine v4rglwindow v4rtracker)
	target_link_libraries(evalRecognise CModelRecogniser v4rpclutils2 v4rpclfunctions2 v4rplanepopout2 EllCalib ${PCL_LIBRARIES})

ENDIF(APPS_TESTS4RECOGNISER)
