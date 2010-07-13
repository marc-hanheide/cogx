
# This option is added so that various interfaces can be built for Python
# compontnes. The support for Python can be turned off if it is not needed
# or when slice2py is not available.
option(BUILD_PYTHON_COMPONENTS "Build Python Ice Interfaces and Components" YES)

set(PYTHON_SLICEGEN_DIR  dist-slicegen)
add_subdirectory(tools/python)

