find_package(PkgConfig REQUIRED)
pkg_search_module(V4R REQUIRED v4r)

MESSAGE(STATUS "V4R_LIBRARY_DIRS: " ${V4R_LIBRARY_DIRS})
MESSAGE(STATUS "V4R_LIBRARIES: " ${V4R_LIBRARIES})
MESSAGE(STATUS "V4R_INCLUDE_DIRS: " ${V4R_INCLUDE_DIRS})