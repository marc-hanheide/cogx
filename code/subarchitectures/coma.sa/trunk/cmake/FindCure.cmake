#
# Find the omniORB libraries and include dir
#
 
# CURE_INSTALLROOT  - The root of the cosycure installation (arg to --prefix)
# CURE_FOUND        - When false, don't try to use omniORB
# CURE_ROOT         - (optional) Suggested installation directory to search
#
# CURE_ROOT can be used to make it simpler to find the various include
# directories and compiled libraries when omniORB was not installed in the
# usual/well-known directories (e.g. because you made an in tree-source
# compilation or because you installed it in an "unusual" directory).
# Just set CURE_DIR it to your specific installation directory

# Try to find the omniidl compiler. 
FIND_PATH(CURE_INSTALLROOT
  NAMES include/cure/config.h #nah: changed to be platform indepdendent (rather than .so .dylib for libs)
  PATHS $ENV{CURE_ROOT}
	/usr/local/
	/opt
	/opt/local
	/Users/luser/packages
  DOC "What is the path where libcosycure can be found minus /lib/cure at the end"
)

IF (CURE_INSTALLROOT)
  SET (CURE_FOUND "YES")
ENDIF (CURE_INSTALLROOT)

IF (NOT CURE_INSTALLROOT)
  MESSAGE("The cosycure installation was not found.")
  MESSAGE("You might want to try and set the CURE_ROOT env variable")
  MESSAGE("It should point to where cosycure was installed, i.e.")
  MESSAGE("  such that $CURE_ROOT/lib/cure will be the place")
  MESSAGE("  where the cure linraries are found for example")
  MESSAGE("You can also set")
  MESSAGE("  - through the GUI when working with ccmake, ")
  MESSAGE("  - as a command line argument when working with cmake e.g. ")
  MESSAGE("    cmake .. -DCURE_INSTALLROOT=/opt/cure ")
  SET(CURE_INSTALLROOT "" CACHE PATH "The path to cosycure installation." )
ENDIF (NOT CURE_INSTALLROOT)

