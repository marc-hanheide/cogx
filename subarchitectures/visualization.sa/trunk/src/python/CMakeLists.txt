project(PythonDisplayClient)

add_subdirectory(dist-slicegen)
add_subdirectory(castmodule)

install(
   DIRECTORY dist-packages/cogxv11n DESTINATION ${PYTHON_INSTALL_PACKAGES}
   PATTERN ".svn" EXCLUDE
   PATTERN "CMakeLists.txt" EXCLUDE
   PERMISSIONS OWNER_WRITE OWNER_READ GROUP_WRITE GROUP_READ
   )

