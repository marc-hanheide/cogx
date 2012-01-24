find_package(PkgConfig REQUIRED)
pkg_search_module(EIGEN3 REQUIRED eigen3)

FIND_PATH(
  UMFPACK_INCLUDE_DIRS umfpack.h
  /usr/include/suitesparse
  /usr/include/ufsparse
  /usr/local/include/suitesparse
  /usr/local/include/ufsparse
  /opt/local/include/suitesparse
  /opt/local/include/ufsparse
)

IF(UMFPACK_INCLUDE_DIRS)
  INCLUDE_DIRECTORIES(${UMFPACK_INCLUDE_DIRS})
  ADD_DEFINITIONS(-DEIGEN_UMFPACK_SUPPORT)
  MESSAGE(STATUS "Found UMFPACK: ${UMFPACK_INCLUDE_DIRS}")
ELSE(UMFPACK_INCLUDE_DIRS)
  MESSAGE(FATAL_ERROR "UMFPACK not found")
ENDIF(UMFPACK_INCLUDE_DIRS)
