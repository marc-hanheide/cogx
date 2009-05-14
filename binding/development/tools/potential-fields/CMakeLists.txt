IF (NOT COSY_ROOT)

   # Set the CoSy root.
   GET_FILENAME_COMPONENT(COSY_ROOT ../.. ABSOLUTE)

   # Set the output directory.
   SET(OUTPUT ${COSY_ROOT}/output)

ENDIF (NOT COSY_ROOT)

# Set the install directory.
SET(CMAKE_INSTALL_PREFIX ${OUTPUT} CACHE INTERNAL "")

# Set the path to additional CMake modules (e.g., our FindOmniORB module).
SET(CMAKE_MODULE_PATH ${COSY_ROOT}/tools/CMakeModules)

# Set the path where other thing should be relative to
GET_FILENAME_COMPONENT(CMAKE_SOURCE_DIR . ABSOLUTE CACHE INTERNAL "")

# Set c++ and idl root.
SET(CPP_ROOT ${CMAKE_SOURCE_DIR}/src/c++)

# Make sure to include all files from cosy.
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/c++)
INCLUDE_DIRECTORIES(${OUTPUT}/include)
LINK_DIRECTORIES(${OUTPUT}/lib)

# You can prepare verbose Makefile to see what's going.
#SET(CMAKE_VERBOSE_MAKEFILE true)


FIND_PACKAGE(OpenGL)
INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})
LINK_LIBRARIES(${OPENGL_LIBRARIES})

FIND_PACKAGE(GLUT)
INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIR})
LINK_LIBRARIES(${GLUT_LIBRARIES})


# Specify the list of subdirectories to process.
ADD_SUBDIRECTORY(src/c++)
ADD_SUBDIRECTORY(src/c++/jni)
