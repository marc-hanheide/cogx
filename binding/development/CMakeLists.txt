
cmake_minimum_required(VERSION 2.6)

if(COMMAND cmake_policy)
       cmake_policy(SET CMP0003 NEW)
     endif(COMMAND cmake_policy)

SET(CMAKE_FIND_LIBRARY_PREFIXES "lib")
SET(CMAKE_FIND_LIBRARY_SUFFIXES "so" "a" "dylib")


FIND_PACKAGE(Boost)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${Boost_LINK_DIRS})


IF (NOT COSY_ROOT)
   # Set the CoSy root.
   GET_FILENAME_COMPONENT(COSY_ROOT . ABSOLUTE)
   # Set the output directory.
   SET(OUTPUT ${COSY_ROOT}/output)
ENDIF (NOT COSY_ROOT)


#always build in debug mode
SET(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE) 


### ADD_DEFINITIONS(-Wall -D_GLIBCXX_DEBUG)
ADD_DEFINITIONS(-Wall)

# Specify the list of subdirectories to process.
#ADD_COSY_DIR(tools/balt)
add_subdirectory(tools/cast)
#ADD_COSY_DIR(tools/matlab-helper)
#ADD_COSY_DIR(tools/CoSyCommonIDLs)
add_subdirectory(subarchitectures/binder)

# Specify the list of subdirectories to process.
#ADD_COSY_DIR(subarchitectures/comsys.mk4) # only for the idls
#ADD_COSY_DIR(subarchitectures/binding.sa)
#ADD_COSY_DIR(subarchitectures/planning.sa)

#ADD_COSY_DIR(subarchitectures/motivation)

#ADD_COSY_DIR(subarchitectures/vision)

#ADD_COSY_DIR(tools/potential-fields)
#ADD_COSY_DIR(subarchitectures/spatial.sa)

#ADD_COSY_DIR(subarchitectures/manipulation.sa)

#ADD_COSY_DIR(subarchitectures/playmate)
#ADD_COSY_DIR(subarchitectures/explorer)


