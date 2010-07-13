project(PythonDisplayClient)

add_subdirectory(dist-slicegen)

install(
   DIRECTORY dist-packages/cogxv11n DESTINATION ${CMAKE_INSTALL_PREFIX}/python/dist-packages
   PATTERN ".svn" EXCLUDE
   PATTERN "CMakeLists.txt" EXCLUDE
   PERMISSIONS OWNER_WRITE OWNER_READ GROUP_WRITE GROUP_READ
   )

install(
   DIRECTORY ./castmodule DESTINATION ${CMAKE_INSTALL_PREFIX}/python
   PATTERN ".svn" EXCLUDE
   PATTERN "CMakeLists.txt" EXCLUDE
   PERMISSIONS OWNER_WRITE OWNER_READ GROUP_WRITE GROUP_READ
   )
