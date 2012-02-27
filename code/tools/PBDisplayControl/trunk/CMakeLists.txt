PROJECT(PBDisplayControl)
find_package(PkgConfig)

pkg_check_modules(GTKMM gtkmm-2.4)

find_package(Peekabot REQUIRED)
include_directories(${PEEKABOT_INSTALLROOT}/include)
link_directories(${PEEKABOT_INSTALLROOT}/lib)

link_directories(
    ${GTKMM_LIBRARY_DIRS}  )

include_directories(
    ${GTKMM_INCLUDE_DIRS}  )

add_executable(PBDisplayControl
    PBDisplayControl.cpp
    PBDisplayControl.hpp )

target_link_libraries(PBDisplayControl 
    ${GTKMM_LIBRARIES} peekabot )

# Installing
INSTALL(TARGETS PBDisplayControl RUNTIME DESTINATION bin)

