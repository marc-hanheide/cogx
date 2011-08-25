## Use precompiled binaries from SVN for FeatureLearning
project(VisualLearnerPrebuilt)
if(UNIX)
   execute_process(
      COMMAND getconf LONG_BIT
      OUTPUT_VARIABLE SYS_ARCH
      OUTPUT_STRIP_TRAILING_WHITESPACE
      )
   if (NOT SYS_ARCH)
      SET(SYS_ARCH 32)
   endif(NOT SYS_ARCH)
   if (SYS_ARCH EQUAL 64)
      get_filename_component(PREBUILT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/glnxa64 ABSOLUTE)
   else(SYS_ARCH EQUAL 64)
      get_filename_component(PREBUILT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/glnx86 ABSOLUTE)
   endif(SYS_ARCH EQUAL 64)
elseif(WIN32)
   get_filename_component(PREBUILT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/win32  ABSOLUTE)
endif(UNIX)

set (VISUALLEARNER_LIBRARY_DIRS ${VISUALLEARNER_LIBRARY_DIRS} ${PREBUILT_DIR} PARENT_SCOPE)
set (LIBRARY_NAME VisualLearner)
set (CTF_EXISTS)
set (SOLIB_EXISTS)

if(UNIX)
   file(GLOB CTF_SOLIB   ${PREBUILT_DIR}/lib${LIBRARY_NAME}Ctf.so)
   file(GLOB CTF_SOPROXY ${PREBUILT_DIR}/lib${LIBRARY_NAME}Proxy.so)
   file(GLOB CTF_ARCHIVE ${PREBUILT_DIR}/lib${LIBRARY_NAME}Ctf.ctf)
   set (OUTLIBRARIES ${CTF_SOLIB} ${CTF_SOPROXY} ${CTF_ARCHIVE})
   set (OUTRUNTIMES)

   set (CTF_EXISTS    ${CTF_ARCHIVE})
   set (SOLIB_EXISTS  ${CTF_SOLIB})
endif(UNIX)

if (SOLIB_EXISTS)
   install(FILES   ${OUTLIBRARIES} DESTINATION lib)
   if (OUTRUNTIMES)
      install(FILES   ${OUTRUNTIMES} DESTINATION lib)
   endif(OUTRUNTIMES)
else (SOLIB_EXISTS)
   file(GLOB SOLIB_INSTALLED   ${OUTPUT}/lib/lib${LIBRARY_NAME}Ctf.so )

   if (NOT SOLIB_INSTALLED)
      set(ZIP_NAME "libVisualLearnerCtf${SYS_ARCH}.zip")
      set(URL_PREBUILT "http://cogx.eu/data/cogx/prebuilt/${ZIP_NAME}")
      message(
         "${LIBRARY_NAME}: Prebuilt Matlab Libraries (CTF) not found.\n"
         "   You can download them from:\n"
         "      ${URL_PREBUILT}\n"
         "   and unpack them into:\n"
         "      ${PREBUILT_DIR}\n"
         "   Then run 'ccmake' again.\n"
         "Commands: \n"
         "   wget ${URL_PREBUILT}\n"
         "   unzip -j  ${ZIP_NAME} -d ${PREBUILT_DIR}\n"
         "If you have Matlab installed you can also build the libraries.\n"
         "   See: BUILD_VISUAL_LEARNER_CTF (Advanced)\n"
      )
   endif (NOT SOLIB_INSTALLED)
endif (SOLIB_EXISTS)

