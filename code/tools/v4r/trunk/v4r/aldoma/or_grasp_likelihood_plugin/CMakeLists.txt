cmake_minimum_required (VERSION 2.6)
project (or_grasp_likelihood_plugin)
find_package(OpenRAVE REQUIRED)
include_directories(${OpenRAVE_INCLUDE_DIRS} /usr/local/include/openrave-0.4)
link_directories(${OpenRAVE_LIBRARY_DIRS})

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_library(or_grasp_likelihood_plugin SHARED src/or_grasp_likelihood_plugin.cpp)
set_target_properties(or_grasp_likelihood_plugin PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(or_grasp_likelihood_plugin ${OpenRAVE_LIBRARIES} vtkRendering vtkGraphics vtkImaging vtkFiltering vtkCommon vtkHybrid)