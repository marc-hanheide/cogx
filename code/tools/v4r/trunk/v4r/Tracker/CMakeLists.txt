SET(V4R_TRACKER 1 CACHE BOOL "Model-based 6-dof object tracker. (depends: TomGine)" )
IF(V4R_TRACKER)

SET(TRACKER_CPP
	CDataFile.cpp
	Distribution.cpp
	EdgeTracker.cpp
	ModelEntry.cpp
	ModelLoader.cpp
	myPredictor.cpp
	Noise.cpp
	Particle.cpp
	ply.c
	Predictor.cpp
	Resources.cpp
	TextureTracker.cpp
	Timer.cpp
	Tracker.cpp
	TrackerModel.cpp)
add_library(v4rtracker SHARED  ${TRACKER_CPP})
target_link_libraries(v4rtracker GL GLU GLEW)
target_link_libraries(v4rtracker v4rtomgine)
target_link_libraries(v4rtracker ${OpenCV_LIBS})
add_subdirectory(example)

ENDIF(V4R_TRACKER)
