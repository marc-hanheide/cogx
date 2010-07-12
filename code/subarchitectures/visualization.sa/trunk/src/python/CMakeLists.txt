project(PythonDisplayClient)

set( slice2py_command   ${ICE_HOME}/bin/slice2py${EXE_EXTENSION} )
set( slice2py_include
   ${COGX_ROOT}/tools/math/src/slice
   ${COGX_ROOT}/tools/hardware/video/src/slice
   /usr/share/slice
   /usr/local/share
   )

add_custom_command(
    TARGET PythonDisplayServerIce
    POST_BUILD
    DEPENDS CDL
    COMMAND ${slice2py_command} ${slice2py_include} --output-dir ./autogen ../slice/DisplayServer.ice
    COMMENT "Generate Display Server Interface for Python"
    )
