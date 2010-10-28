#
# Find the peekabot libraries and include dir
#
 
# PEEKABOT_INSTALLROOT  - The root of the cosycure installation (arg to --prefix)
# PEEKABOT_FOUND        - When false, don't try to use omniORB
# PEEKABOT_ROOT         - (optional) Suggested installation directory to search
#
# PEEKABOT_ROOT can be used to make it simpler to find the various include
# directories and compiled libraries when peekabot was not installed in the
# usual/well-known directories (e.g. because you made an in tree-source
# compilation or because you installed it in an "unusual" directory).
# Just set PEEKABOT_DIR it to your specific installation directory

# Try to find the omniidl compiler. 
FIND_PATH(PEEKABOT_INSTALLROOT
  NAMES include/peekabot/client/proxies/LabelProxy.hh
  PATHS $ENV{PEEKABOT_ROOT}
        $ENV{HOME}/peekabot/
	/usr/local/
	/usr/
	/Users/luser/packages
  DOC "What is the path where libcosycure can be found minus /lib/cure at the end"
)

IF (PEEKABOT_INSTALLROOT)
  SET (PEEKABOT_FOUND "YES")
ENDIF (PEEKABOT_INSTALLROOT)

IF (NOT PEEKABOT_INSTALLROOT)
  MESSAGE("The peekabot installation was not found.")
  MESSAGE("You might want to try and set the PEEKABOT_ROOT env variable")
  MESSAGE("It should point to where cosycure was installed, i.e.")
  MESSAGE("  such that $PEEKABOT_ROOT/lib/cure will be the place")
  MESSAGE("  where the cure linraries are found for example")
  MESSAGE("You can also set")
  MESSAGE("  - through the GUI when working with ccmake, ")
  MESSAGE("  - as a command line argument when working with cmake e.g. ")
  MESSAGE("    cmake .. -DPEEKABOT_INSTALLROOT=/opt/cure ")
  SET(PEEKABOT_INSTALLROOT "" CACHE PATH "The path to peekabot installation." )
ENDIF (NOT PEEKABOT_INSTALLROOT)

