
IF (WIN32)
	MESSAGE ("NOT Available")
ELSE (WIN32)
	FIND_PATH( LIBPLOT_INCLUDE_PATH plot.h
		PATHS
		/usr/include
		/usr/local/include
		/sw/include
		/opt/local/include
		DOC "The directory where plotter.h resides")
	SET(LIBPATHS
		/usr/lib64
		/usr/lib
		/usr/local/lib64
		/usr/local/lib
		/sw/lib
		/opt/local/lib
		)
	FIND_LIBRARY( LIBPLOT_PLOT_LIBRARY plot
		NAMES plot
		PATHS ${LIBPATHS}
		DOC "The plotutils libplot library")
	FIND_LIBRARY( LIBPLOT_PLOTTER_LIBRARY plotter
		NAMES plotter
		PATHS ${LIBPATHS}
		DOC "The plotutils libplotter library")
ENDIF (WIN32)

IF (LIBPLOT_INCLUDE_PATH)
	SET(FOUND 1)
ELSE (LIBPLOT_INCLUDE_PATH)
	SET(FOUND 0)
ENDIF (LIBPLOT_INCLUDE_PATH)

SET( LIBPLOT_FOUND ${FOUND} CACHE STRING "Set to 1 if LIBPLOT is found, 0 otherwise")
MARK_AS_ADVANCED( LIBPLOT_FOUND )
