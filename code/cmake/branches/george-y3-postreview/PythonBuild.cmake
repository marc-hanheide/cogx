
# This option is added so that various interfaces can be built for Python
# compontnes. The support for Python can be turned off if it is not needed
# or when slice2py is not available.
option(BUILD_PYTHON_COMPONENTS "Build Python Ice Interfaces and Components" YES)

set(PYTHON_INSTALL_PREFIX     ${CMAKE_INSTALL_PREFIX}/python)
set(PYTHON_INSTALL_PACKAGES   ${PYTHON_INSTALL_PREFIX}/dist-packages)
set(PYTHON_INSTALL_SLICEGEN   ${PYTHON_INSTALL_PREFIX}/dist-slicegen)
set(PYTHON_INSTALL_CASTMODULE ${PYTHON_INSTALL_PREFIX}/castmodule)

add_subdirectory(tools/python)

