## Build the matlab CTF/shared library for the VisualLearner component
## of the vision subarchitecture.
##
## Outline:
##      1. prepare the project file for MCC
##      2. run MCC -F project file
##      3. install the resulting CTF/SO/DLL/LIB/H to destination
##
## When mcc/mbuild is not available, try to use precompiled binaries from SVN

SET(CAN_BUILD_CTF "No")
IF (BUILD_SA_VISION_VISLEARNER_CTF)
   IF (MATLAB_BIN_MCC AND MATLAB_BIN_MBUILD)
      SET(CAN_BUILD_CTF "Yes")
   ENDIF (MATLAB_BIN_MCC AND MATLAB_BIN_MBUILD)
ENDIF (BUILD_SA_VISION_VISLEARNER_CTF)


PROJECT(VisualLearnerCtf)
SET (LIBRARY_NAME VisualLearnerCtf)
GET_FILENAME_COMPONENT(MSOURCE_DIR .. ABSOLUTE)
GET_FILENAME_COMPONENT(MCCDIST_DIR ${CMAKE_CURRENT_BINARY_DIR}/distrib  ABSOLUTE)
IF(CAN_BUILD_CTF)
   GET_FILENAME_COMPONENT(MCCBUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/build  ABSOLUTE)
ELSE(CAN_BUILD_CTF)
   MESSAGE(SEND_ERROR  "Visual Learner CTF cannot be built.")
   # precompiled binaries from SVN
   IF(UNIX)
      GET_FILENAME_COMPONENT(MCCBUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin/glnx86 ABSOLUTE)
   ELSEIF(WIN32)
      GET_FILENAME_COMPONENT(MCCBUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin/win32  ABSOLUTE)
   ENDIF(UNIX)
ENDIF(CAN_BUILD_CTF)

IF(UNIX)
   SET(MCCPROJECT_NAME lib${LIBRARY_NAME})
   SET(OUTHEADERS ${MCCBUILD_DIR}/${MCCPROJECT_NAME}.h)
   SET(OUTLIBRARIES
      ${MCCBUILD_DIR}/${MCCPROJECT_NAME}.ctf
      ${MCCBUILD_DIR}/${MCCPROJECT_NAME}.so
   )
   SET(OUTRUNTIMES)
ELSEIF(WIN32)
   SET(MCCPROJECT_NAME ${LIBRARY_NAME})
   SET(OUTHEADERS ${MCCBUILD_DIR}/${MCCPROJECT_NAME}.h)
   SET(OUTLIBRARIES
      ${MCCBUILD_DIR}/${MCCPROJECT_NAME}.lib
   )
   SET(OUTRUNTIMES
      ${MCCBUILD_DIR}/${MCCPROJECT_NAME}.ctf
      ${MCCBUILD_DIR}/${MCCPROJECT_NAME}.dll
   )
ENDIF(UNIX)
SET(MCC_PROJECT  ${CMAKE_CURRENT_BINARY_DIR}/${MCCPROJECT_NAME}.prj)
SET(MCC_OUTPUT   ${OUTHEADERS} ${OUTLIBRARIES} ${OUTRUNTIMES} )

IF(CAN_BUILD_CTF)
   ## Create the project file to be used by MCC
   FIND_PROGRAM(CMAKE_PYTHON python)
   ADD_CUSTOM_COMMAND(OUTPUT ${MCC_PROJECT}
         COMMAND ${CMAKE_PYTHON}
            makeproject.py 
            --matlabversion 780 
            --mscriptroot ${MSOURCE_DIR}
            --libraryname  ${MCCPROJECT_NAME}
            --projectroot ${CMAKE_CURRENT_BINARY_DIR}
            --buildoutput ${MCCBUILD_DIR}
            --distriboutput ${MCCDIST_DIR}
            --export mfile-export.txt
#~             --cppexport cfile-export.txt
         WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
         DEPENDS
            makeproject.py 
            deployment.prj.in
            matlab.dir.in
            mfile-export.txt
   )

   FILE(GLOB_RECURSE MSOURCE_FILES
      ${MSOURCE_DIR}/*.m
   )

   # MESSAGE(">>>" ${MSOURCE_FILES})

   ## Run MCC to create ctf/library in the MCC-build directory
   ADD_CUSTOM_COMMAND(OUTPUT ${MCC_OUTPUT}
      COMMAND mkdir -p ${MCCBUILD_DIR}
   #~    COMMAND mkdir -p ${MCCDIST_DIR}
      COMMAND mcc -F ${MCC_PROJECT}
      DEPENDS ${MCC_PROJECT}
         ${MSOURCE_FILES}
   )

   ADD_CUSTOM_TARGET(MccVisualLearnerCtf ALL
      DEPENDS ${MCC_OUTPUT}
   )
ENDIF(CAN_BUILD_CTF)

INSTALL(FILES   ${OUTLIBRARIES} 
   DESTINATION lib
)
#~ INSTALL(FILES   ${OUTHEADERS}
#~    DESTINATION include/vision/matlab
#~ )
IF (OUTRUNTIMES)
   INSTALL(FILES   ${OUTRUNTIMES}
      DESTINATION bin
   )
ENDIF(OUTRUNTIMES)

# Clear the value and find the file again
#~ SET(FEATURE_LEARNING_CTF_HEADER "FEATURE_LEARNING_CTF_HEADER-NOTFOUND"
#~    CACHE FILEPATH "Header required for FeatureLearning2"
#~    FORCE
#~ )
#~ FIND_FILE(FEATURE_LEARNING_CTF_HEADER
#~    NAME ${MCCPROJECT_NAME}.h
#~    PATHS ${OUTPUT}/include/vision/matlab
#~    NO_DEFAULT_PATH
#~    DOC "Header required for FeatureLearning2"
#~ )
#~ MARK_AS_ADVANCED(FORCE FEATURE_LEARNING_CTF_HEADER)
