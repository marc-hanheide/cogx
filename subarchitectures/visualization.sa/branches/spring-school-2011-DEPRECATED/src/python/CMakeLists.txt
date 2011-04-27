project(PythonDisplayClient)



if(BUILD_SA_V11N_AUTOGEN_ONLY)

  add_subdirectory(dist-slicegen)

else(BUILD_SA_V11N_AUTOGEN_ONLY)

  add_subdirectory(dist-slicegen)
  add_subdirectory(castmodule)

endif(BUILD_SA_V11N_AUTOGEN_ONLY)


install(
   DIRECTORY dist-packages/cogxv11n DESTINATION ${PYTHON_INSTALL_PACKAGES}
   PATTERN ".svn" EXCLUDE
   PATTERN "CMakeLists.txt" EXCLUDE
   PERMISSIONS OWNER_WRITE OWNER_READ GROUP_WRITE GROUP_READ
   )

