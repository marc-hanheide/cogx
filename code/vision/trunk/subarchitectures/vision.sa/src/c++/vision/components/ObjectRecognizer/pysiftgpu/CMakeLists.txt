cmake_minimum_required(VERSION 2.6)

project(PySiftGPU)

find_package(PythonLibs)
find_package(Boost)
set(SIFTGPU_INCLUDE 
   ${CMAKE_CURRENT_SOURCE_DIR}/../libsiftgpu/SiftGPU/src
   /usr/include/GL
   )

include_directories(SYSTEM ${PYTHON_INCLUDE_PATH} ${Boost_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR} ${SIFTGPU_INCLUDE})

link_directories(/usr/lib Boost_LIBRARY_DIRS)

# -----------------------------------------------------------------
# Commands and targets for wrapper generation.
# Wrapper generation systems: Py++ or pyste.
# -----------------------------------------------------------------
set(select_wrapper "pyplusplus")

if (${select_wrapper} STREQUAL "pyplusplus")
   set(siftgpu_wrapper pypp/xdata/bindings.cpp)
   add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/${siftgpu_wrapper}
      COMMAND python makewrapper.py
         --basedir "${CMAKE_CURRENT_SOURCE_DIR}"
         --siftgpudir  "${CMAKE_CURRENT_SOURCE_DIR}/../libsiftgpu"
         --pyinclude "${PYTHON_INCLUDE_PATH}"
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/pypp
      DEPENDS pypp/makewrapper.py pypp/wrap_*.cpp
      )
elseif (${select_wrapper} STREQUAL "pyste")
   set(siftgpu_wrapper pyste/_siftgpuproxy_pywrap.cpp)
   set(extra_sources "SiftGpuProxy.cpp")
   add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/${siftgpu_wrapper}
     COMMAND pyste 
        -I${PYTHON_INCLUDE_PATH} -I${Boost_INCLUDE_DIR} -I${SIFTGPU_INCLUDE} -I..
        --module=SiftGpu --out=../${siftgpu_wrapper} 
        SiftGpuProxy.pyste
     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/pyste
     DEPENDS pyste/SiftGpuProxy.pyste
     )
endif (${select_wrapper} STREQUAL "pyplusplus")

# -----------------------------------------------------------------
# Python module extra initialization
# -----------------------------------------------------------------
add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/__init__.py
   COMMAND cp 
      ${CMAKE_CURRENT_SOURCE_DIR}/siftgpu__init.py 
      ${CMAKE_CURRENT_BINARY_DIR}/__init__.py
   DEPENDS siftgpu__init.py
   )

add_custom_target(PythonInitModules
   DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/__init__.py
   )
# -----------------------------------------------------------------
# The library
# -----------------------------------------------------------------
set(LIBSIFTGPU __PySiftGpu)
add_library(${LIBSIFTGPU} SHARED
   ${siftgpu_wrapper}
   ${extra_sources} # SiftGpuProxy.cpp # TODO: is this really used or was it replaced by Python code?
   )
set_target_properties(${LIBSIFTGPU} PROPERTIES PREFIX "")
add_dependencies(${LIBSIFTGPU} ${siftgpu_wrapper} PythonInitModules)
target_link_libraries(${LIBSIFTGPU} 
   Cg CgGL GLEW glut IL # OpenGL etc.
   boost_python # Boost
   siftgpu # SiftGPU shared library that we are wrapping, separate project
   )

# -----------------------------------------------------------------
# Installation
# -----------------------------------------------------------------
set(pymodule "siftgpu")
install(TARGETS ${LIBSIFTGPU} 
   DESTINATION ${COGX_ROOT}/output/python/${pymodule})
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/__init__.py
   DESTINATION ${COGX_ROOT}/output/python/${pymodule})

