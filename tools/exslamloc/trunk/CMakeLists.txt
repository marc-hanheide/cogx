cmake_minimum_required (VERSION 2.6)

PROJECT(exslamloc)

# add_subdirectory (src/c++/autogen)
if (DO_SLICE_ONLY)
   return()
endif()

find_package(PkgConfig)

pkg_check_modules(GTKMM gtkmm-2.4)

find_package(Player REQUIRED)

find_package(Peekabot REQUIRED)
include_directories(${PEEKABOT_INSTALLROOT}/include)
link_directories(${PEEKABOT_INSTALLROOT}/lib)

find_package(Cure REQUIRED)
include_directories(${CURE_INSTALLROOT}/include/cure)
link_directories(${CURE_INSTALLROOT}/lib/cure)

link_directories(
    ${GTKMM_LIBRARY_DIRS}  )

include_directories(
    ${GTKMM_INCLUDE_DIRS}  )

add_executable(exslamloc
    exslamloc.cc PlayerDataToCure.cc LsqLineExtractor.cc DoorExtractor.cc  )

target_link_libraries(exslamloc 
    ${GTKMM_LIBRARIES} playerc++ peekabot CureUtils CureMath CureTransformation CureGeometry CureSensorData CureSensory CureAddressBank CureFilters CureMSpace CureNav CureMapBase CureMapWall CureMapPoint CureMapHLine CureMapRangedPoint CureRelativePointFeature CureMapApps CureNavX CureMapX)

# Installing
INSTALL(TARGETS exslamloc RUNTIME DESTINATION bin)

