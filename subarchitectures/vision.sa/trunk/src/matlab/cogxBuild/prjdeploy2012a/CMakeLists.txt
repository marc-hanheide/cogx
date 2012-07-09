# in R2012a mcc does not support -F option any more and the project has to be
# built from within Matlab using the deploytool.
#
# In this project we only prepare a Deploy Tool Project File. It will be
# prepared in 'old' format, but Matlab will convert it on first use.

find_package(MatlabCompiler)
#include_directories( ${MATLAB_INCLUDE_DIR} )

PROJECT(VisualLearnerCtf)
#
# The CTF file can be either separate or included in the executable (so/dll).
SET(SEPARATE_CTF "false")

SET (LIBRARY_NAME VisualLearnerCtf)
GET_FILENAME_COMPONENT(MSOURCE_DIR ../.. ABSOLUTE)
GET_FILENAME_COMPONENT(MCCDIST_DIR ${CMAKE_CURRENT_BINARY_DIR}/distrib  ABSOLUTE)
#IF(CAN_BUILD_CTF)
   GET_FILENAME_COMPONENT(MCCBUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/build  ABSOLUTE)
#ELSE(CAN_BUILD_CTF)
#   MESSAGE(SEND_ERROR  "Visual Learner CTF cannot be built.")
#   # precompiled binaries from SVN
#   GET_FILENAME_COMPONENT(MCCBUILD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/bin/glnx86 ABSOLUTE)
#ENDIF(CAN_BUILD_CTF)

SET(MCCPROJECT_NAME lib${LIBRARY_NAME})
SET(OUTHEADERS ${MCCBUILD_DIR}/${MCCPROJECT_NAME}.h)
SET(OUTLIBRARIES ${MCCBUILD_DIR}/${MCCPROJECT_NAME}.so)
IF (SEPARATE_CTF EQUAL "true")
   SET(OUTLIBRARIES ${OUTLIBRARIES} ${MCCBUILD_DIR}/${MCCPROJECT_NAME}.ctf)
ENDIF(SEPARATE_CTF EQUAL "true")
SET(OUTRUNTIMES)

SET(MCC_PROJECT  ${CMAKE_CURRENT_BINARY_DIR}/${MCCPROJECT_NAME}.prj)
SET(MCC_OUTPUT   ${OUTHEADERS} ${OUTLIBRARIES} ${OUTRUNTIMES} )
SET(MCC_MAKEFILE ${CMAKE_CURRENT_BINARY_DIR}/${MCCPROJECT_NAME}.mak)

## Create the project file to be used by MCC
FIND_PROGRAM(CMAKE_PYTHON python)

add_custom_command(OUTPUT ${MCC_MAKEFILE}
   COMMAND ${CMAKE_PYTHON}
      makebuildscript.py 
      --export mfile-export.txt
      --libraryname  ${MCCPROJECT_NAME}
      --mscriptroot  ${MSOURCE_DIR}
      --buildroot    ${CMAKE_CURRENT_BINARY_DIR}
      --buildoutput  ${CMAKE_CURRENT_BINARY_DIR}/build
      --separatectf  ${SEPARATE_CTF}
      --makefile
   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
   DEPENDS
      makebuildscript.py 
      mccbuild.mak.in
      mfile-export.txt
   )

add_custom_target(MccVisualLearnerCtf ALL
   DEPENDS ${MCC_MAKEFILE}
   COMMAND make -f ${MCC_MAKEFILE}
   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
   )

#configure_file(buildvisuallearnerctf.sh.in buildvisuallearnerctf.sh)
configure_file(cpvisuallearnerprebuilt.sh.in cpvisuallearnerprebuilt.sh)
install(PROGRAMS
   #${CMAKE_CURRENT_BINARY_DIR}/buildvisuallearnerctf.sh
   ${CMAKE_CURRENT_BINARY_DIR}/cpvisuallearnerprebuilt.sh
   DESTINATION ${OUTPUT}/bin
   )

