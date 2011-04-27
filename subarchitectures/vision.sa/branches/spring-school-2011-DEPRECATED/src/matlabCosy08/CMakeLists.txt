find_package(MatlabCompiler)
include_directories( ${MATLAB_INCLUDE_DIR} )

add_subdirectory(prjdeploy)
# add_subdirectory(IceBackend)
add_subdirectory(Proxy)
