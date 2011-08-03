cmake_minimum_required (VERSION 2.6)

set(CMAKE_BUILD_TYPE Release)

# ------------ <CCache> ------------
if(USE_CCACHE)
        set(CMAKE_CXX_COMPILER "ccache")
        set(CMAKE_CXX_COMPILER_ARG1 "g++")
        set(CMAKE_C_COMPILER "ccache")
        set(CMAKE_C_COMPILER_ARG1 "gcc")
endif(USE_CCACHE)
# ------------ </CCache> ------------

project (CogX)

#message(${CMAKE_CXX_COMPILER})
get_filename_component(COGX_ROOT . ABSOLUTE)

# set the output directory.
set(OUTPUT ${COGX_ROOT}/output CACHE PATH "Path prefix for installation")
set(CMAKE_INSTALL_PREFIX "${OUTPUT}" CACHE INTERNAL "Prefix prepended to install directories" FORCE)


# use cmake files packaged with CAST as well
set(CMAKE_MODULE_PATH /usr/share/cast/cmake ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH /usr/local/share/cast/cmake ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH /opt/local/share/cast/cmake ${CMAKE_MODULE_PATH})
set(QT_QTOPENGL_INCLUDE_DIR  /usr/include/qt4/QtOpenGL)

find_package(CAST REQUIRED)
include(UseCAST)
include(CASTBuild)

# use cmake files packaged with CAST as well
set(CMAKE_MODULE_PATH "${COGX_ROOT}/cmake;${CAST_CMAKE_DIR}" ${CMAKE_MODULE_PATH})

add_definitions(-Wall)

add_subdirectory (tools/math)
add_subdirectory (tools/d-lib)
add_subdirectory (tools/hardware/pointCloud)
add_subdirectory (tools/hardware/video)
add_subdirectory (tools/hardware/laser)
add_subdirectory (tools/hardware/kinect)
add_subdirectory (tools/hardware/robotbase)
add_subdirectory (tools/hardware/ptz)
add_subdirectory (tools/hardware/utils)
add_subdirectory (subarchitectures/data.sa)
add_subdirectory (subarchitectures/spatial.sa)
