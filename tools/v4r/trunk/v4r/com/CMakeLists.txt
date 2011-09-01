SET(V4R_COM 1 CACHE BOOL "Add description here." )
IF(V4R_COM)

SET( USER "$ENV{USER}" )
IF ( ${USER} STREQUAL "max"  )
add_subdirectory(example)
ENDIF( ${USER} STREQUAL "max"  )

ENDIF(V4R_COM)
