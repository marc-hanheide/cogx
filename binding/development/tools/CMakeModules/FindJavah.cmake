# - Find javah
# This module looks for javah, a C header and stub file generator. Since
# CMake's module FindJava does not set the path to javah executable, we don't
# need to run FindJava prior to this module.
#
# Exposed variables:
# 	JAVAH_FOUND		= this variable is set only if the JAVAH_RUNTIME is found
# 	JAVAH_RUNTIME	= the full path to the javah runtime

SET(JAVA_BIN_PATH
	$ENV{JAVA_HOME}/bin
	/usr/bin
	/usr/lib/java/bin
	/usr/share/java/bin
	/usr/local/bin
	/usr/local/java/bin
	/usr/java/j2sdk1.4.2_04
	/usr/lib/j2sdk1.4-sun/bin
	/usr/java/j2sdk1.4.2_09/bin
	/usr/lib/j2sdk1.5-sun/bin
	/opt/sun-jdk-1.5.0.04/bin
	"[HKEY_LOCAL_MACHINE\\SOFTWARE\\JavaSoft\\Java Development Kit\\2.0;JavaHome]/bin"
	"[HKEY_LOCAL_MACHINE\\SOFTWARE\\JavaSoft\\Java Development Kit\\1.9;JavaHome]/bin"
	"[HKEY_LOCAL_MACHINE\\SOFTWARE\\JavaSoft\\Java Development Kit\\1.8;JavaHome]/bin"
	"[HKEY_LOCAL_MACHINE\\SOFTWARE\\JavaSoft\\Java Development Kit\\1.7;JavaHome]/bin"
	"[HKEY_LOCAL_MACHINE\\SOFTWARE\\JavaSoft\\Java Development Kit\\1.6;JavaHome]/bin"
	"[HKEY_LOCAL_MACHINE\\SOFTWARE\\JavaSoft\\Java Development Kit\\1.5;JavaHome]/bin"
	"[HKEY_LOCAL_MACHINE\\SOFTWARE\\JavaSoft\\Java Development Kit\\1.4;JavaHome]/bin"
	"[HKEY_LOCAL_MACHINE\\SOFTWARE\\JavaSoft\\Java Development Kit\\1.3;JavaHome]/bin"
)

FIND_PATH(JAVAH_INCLUDE_DIR jni.h
	$ENV{JAVA_HOME}/include
	/usr/local/include
	/System/Library/Frameworks/JavaVM.framework/Headers
)

# Try to locate the javah runtime.
FIND_PROGRAM(JAVAH_RUNTIME
	NAMES javah
	PATHS ${JAVA_BIN_PATH}
)

IF (JAVAH_RUNTIME)
	SET (JAVAH_FOUND "YES")

	# Mark the variable as advanced so that users don't need to know about it.
	MARK_AS_ADVANCED(JAVAH_RUNTIME)
	MARK_AS_ADVANCED(JAVAH_INCLUDE_DIR)
ENDIF (JAVAH_RUNTIME)

IF (NOT JAVAH_FOUND)
  MESSAGE("javah was not found in the path. Please provide JAVAH_RUNTIME:")
  MESSAGE("  - through the GUI when working with ccmake, ")
  MESSAGE("  - as a command line argument when working with cmake e.g. ")
  MESSAGE("    cmake .. -DJAVAH_RUNTIME:PATH=/usr/bin/javah ")
  SET(JAVAH_RUNTIME "" CACHE PATH "The path to javah executable." )
ENDIF (NOT JAVAH_FOUND)


