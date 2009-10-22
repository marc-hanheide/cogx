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

set (LIBRARY_NAME VisualLearner)
set (CTF_EXISTS)
if(UNIX)
   set(CTF_ARCHIVE  "lib${LIBRARY_NAME}Ctf.ctf")
   set(CTF_LIB)
   set(CTF_SOLIB    "lib${LIBRARY_NAME}Ctf.so")
   set(CTF_SOPROXY  "lib${LIBRARY_NAME}Proxy.so")
   file(GLOB CTF_EXISTS  ${PREBUILT_DIR}/${CTF_ARCHIVE})
   file(GLOB SOLIB_EXISTS  ${PREBUILT_DIR}/${CTF_SOLIB})
   set(OUTLIBRARIES
      ${PREBUILT_DIR}/${CTF_SOLIB}
      ${PREBUILT_DIR}/${CTF_SOPROXY}
   )
   set(OUTRUNTIMES)
   if(CTF_EXISTS)
      set(OUTLIBRARIES ${OUTLIBRARIES} ${PREBUILT_DIR}/${CTF_ARCHIVE})
   endif(CTF_EXISTS)
elseif(WIN32)
   set(CTF_ARCHIVE  "${LIBRARY_NAME}Ctf.ctf")
   set(CTF_LIB      "${LIBRARY_NAME}Ctf.lib")
   set(CTF_SOLIB    "${LIBRARY_NAME}Ctf.dll")
   set(CTF_SOPROXY  "${LIBRARY_NAME}Proxy.dll")
   set(OUTLIBRARIES
      ${PREBUILT_DIR}/${CTF_LIB}
   )
   file(GLOB CTF_EXISTS  ${PREBUILT_DIR}/${CTF_ARCHIVE})
   file(GLOB SOLIB_EXISTS  ${PREBUILT_DIR}/${CTF_SOLIB})
   set(OUTRUNTIMES
      ${PREBUILT_DIR}/${CTF_SOLIB}
      ${PREBUILT_DIR}/${CTF_SOPROXY}
   )
   if(CTF_EXISTS)
      set(OUTRUNTIMES ${OUTRUNTIMES} ${PREBUILT_DIR}/${CTF_ARCHIVE})
   endif(CTF_EXISTS)
endif(UNIX)

if (SOLIB_EXISTS)
   install(FILES   ${OUTLIBRARIES} DESTINATION lib)
   if (OUTRUNTIMES)
      install(FILES   ${OUTRUNTIMES} DESTINATION lib)
   endif(OUTRUNTIMES)
else (SOLIB_EXISTS)
   if(CTF_ARCHIVE)
      file(GLOB CTF_INSTALLED  ${PREBUILT_DIR}/${CTF_ARCHIVE})
   endif(CTF_ARCHIVE)

   if (NOT CTF_INSTALLED)
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
   endif (NOT CTF_INSTALLED)
endif (SOLIB_EXISTS)

