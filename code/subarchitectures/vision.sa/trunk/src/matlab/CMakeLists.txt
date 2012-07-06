find_package(MatlabCompiler)
include_directories( ${MATLAB_INCLUDE_DIR} )

add_subdirectory(cogxBuild/prjdeploy2012a)
add_subdirectory(cogxBuild/Proxy)
