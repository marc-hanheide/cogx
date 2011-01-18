project(CategoricalPythonTools)

install(DIRECTORY . DESTINATION ${CMAKE_INSTALL_PREFIX}/python/categorical
	PATTERN ".svn" EXCLUDE
	PATTERN "CMakeLists.txt" EXCLUDE
	PATTERN "train_shape.py" EXCLUDE
	PATTERN "train_appearance.py" EXCLUDE
	)

install(PROGRAMS train_shape.py train_appearance.py DESTINATION ${CMAKE_INSTALL_PREFIX}/python/categorical)
