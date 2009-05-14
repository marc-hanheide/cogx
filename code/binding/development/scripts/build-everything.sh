#!/bin/bash


# tool directories to build... built in order listed

ALL_TOOLS_DIRS=$ALL_TOOLS_DIRS" balt" 
ALL_TOOLS_DIRS=$ALL_TOOLS_DIRS" caat"
#ALL_TOOLS_DIRS=$ALL_TOOLS_DIRS" CoSyCommonIDLs"
#ALL_TOOLS_DIRS=$ALL_TOOLS_DIRS" matlab-helper"
#ALL_TOOLS_DIRS=$ALL_TOOLS_DIRS" comedian-architecture"
#ALL_TOOLS_DIRS=$ALL_TOOLS_DIRS" potential-fields"
#ALL_TOOLS_DIRS=$ALL_TOOLS_DIRS" logical-forms"
#ALL_TOOLS_DIRS=$ALL_TOOLS_DIRS" interconnectivity"
#ALL_TOOLS_DIRS=$ALL_TOOLS_DIRS" jracer"
#ALL_TOOLS_DIRS=$ALL_TOOLS_DIRS" openccg"
#ALL_TOOLS_DIRS=$ALL_TOOLS_DIRS" vs2"

# subarchitecture directories to build... built in order listed

ALL_SUBARCH_DIRS=$ALL_SUBARCH_DIRS" benchmark.sa"
#ALL_SUBARCH_DIRS=$ALL_SUBARCH_DIRS" vision"
#ALL_SUBARCH_DIRS=$ALL_SUBARCH_DIRS" category.binding.sa"
# right now compiles only comsys.mk3 and catsys.sa, 
# but it could be expanded to other subarchs if dependencies require it:
#ALL_SUBARCH_DIRS=$ALL_SUBARCH_DIRS" java-subarch-build"
#ALL_SUBARCH_DIRS=$ALL_SUBARCH_DIRS" placeholder.language.sa"
#ALL_SUBARCH_DIRS=$ALL_SUBARCH_DIRS" examples.sa"
#ALL_SUBARCH_DIRS=$ALL_SUBARCH_DIRS" manipulation.sa"
#ALL_SUBARCH_DIRS=$ALL_SUBARCH_DIRS" comsys.mk3"
#ALL_SUBARCH_DIRS=$ALL_SUBARCH_DIRS" spatial.sa"
#ALL_SUBARCH_DIRS=$ALL_SUBARCH_DIRS" planning.sa"
#ALL_SUBARCH_DIRS=$ALL_SUBARCH_DIRS" central.mechanisms"

# instantiation directories to build... built in order listed

#ALL_INSTANT_DIRS=$ALL_INSTANT_DIRS" uol"
#ALL_INSTANT_DIRS=$ALL_INSTANT_DIRS" central.mechanisms.test"
#ALL_INSTANT_DIRS=$ALL_INSTANT_DIRS" visual.learning.integration"
#ALL_INSTANT_DIRS=$ALL_INSTANT_DIRS" spatial.memory.test"


for ARG in "$@"
  do
  if [ "$ARG" = "configure" ] 
      then
      CMAKE_CONFIGURE=true
      echo Configuring code before building
  fi 
  
  if [ "$ARG" = "tools" ] 
      then
      BUILD_TOOLS=true
      echo Building tools
  fi

  if [ "$ARG" = "subarchs" ] 
      then
      BUILD_SUBARCHS=true
      echo Building subachitectures
  fi

  if [ "$ARG" = "instants" ] 
      then
      BUILD_INSTANTS=true
      echo Building instatiations
  fi

  if [ "$ARG" = "install" ] 
      then
      MAKE_ARGUMENT=install
      ANT_ARGUMENT=
  fi

  if [ "$ARG" = "clean" ] 
      then
      MAKE_ARGUMENT=clean
      ANT_ARGUMENT=clean
  fi
  
done


#if not set, run make install
if [ -z "$MAKE_ARGUMENT" ]
    then
    MAKE_ARGUMENT=install
    ANT_ARGUMENT=
fi

# if none set, build all!
if [ -z "$BUILD_TOOLS" ] && [ -z "$BUILD_SUBARCHS" ] && [ -z "$BUILD_INSTANTS" ]
    then 
    BUILD_TOOLS=true
    BUILD_SUBARCHS=true
    BUILD_INSTANTS=true
    echo Building tools
    echo Building subachitectures
    echo Building instatiations
fi


COSY_ROOT=`pwd`
COSY_BUILD_DIR=$COSY_ROOT/build

TOOLS_BUILD_DIR=$COSY_BUILD_DIR/tools
TOOLS_DIR=$COSY_ROOT/tools

SUBARCHS_BUILD_DIR=$COSY_BUILD_DIR/subarchitectures
SUBARCHS_DIR=$COSY_ROOT/subarchitectures

INSTANTS_BUILD_DIR=$COSY_BUILD_DIR/instantiations
INSTANTS_DIR=$COSY_ROOT/instantiations


#CMAKE_CMD=cmake #use for no gui
CMAKE_CMD=ccmake #use for gui config

#CMAKE_BUILD_TYPE=Debug
CMAKE_BUILD_TYPE=Release

mkdir -p $TOOLS_BUILD_DIR
mkdir -p $SUBARCHS_BUILD_DIR
mkdir -p $INSTANTS_BUILD_DIR





if [ -n "$BUILD_TOOLS" ]
    then 
    for TOOL_DIR in $ALL_TOOLS_DIRS
      do
  #ant build first
      ANT_BUILD_FILE="$TOOLS_DIR/$TOOL_DIR/build.xml" 
      
      if [ -a $ANT_BUILD_FILE ]
	  then
	  echo $ANT_BUILD_FILE
	  ant -f $ANT_BUILD_FILE $ANT_ARGUMENT
	  if [ $? -ne 0 ] 
	      then
	      echo Failed when running ant on $ANT_BUILD_FILE
	      echo Test just this with the command
	      echo "    " ant -f $ANT_BUILD_FILE  $ANT_ARGUMENT

	      exit $?
	  fi
      fi

  #now cmake build
      CMAKE_LISTS_FILE="$TOOLS_DIR/$TOOL_DIR/CMakeLists.txt" 
      if [ -a $CMAKE_LISTS_FILE ]
	  then

      #echo $CMAKE_LISTS_FILE
	  BUILD_DIR="$TOOLS_BUILD_DIR/$TOOL_DIR"
	  mkdir -p $BUILD_DIR

	  pushd $BUILD_DIR > /dev/null

	  if [ -n "$CMAKE_CONFIGURE" ] 
	      then
	      $CMAKE_CMD "-DCMAKE_BUILD_TYPE:STRING=$CMAKE_BUILD_TYPE" "$TOOLS_DIR/$TOOL_DIR"
	      if [ $? -ne 0 ] 
		  then
		  echo Failed to configure with $CMAKE_CMD in "$TOOLS_DIR/$TOOL_DIR"
		  echo Test just this with the command
		  echo "    " cd $BUILD_DIR\; $CMAKE_CMD "$TOOLS_DIR/$TOOL_DIR"
		  exit $?
	      fi
	  fi
	  make $MAKE_ARGUMENT
	  if [ $? -ne 0 ] 
	      then
	      echo Failed build for "$TOOLS_DIR/$TOOL_DIR"
	      echo Test just this with the command:
	      echo "    " make -C $BUILD_DIR $MAKE_ARGUMENT
	      exit $?
	  fi
	  popd
      fi
      


  #pushd "$TOOLS_DIRS$TOOL_DIR"  > /dev/null
#  pwd
 # popd > /dev/null
    done
fi

if [ -n "$BUILD_SUBARCHS" ]
    then 

    for SUBARCH_DIR in $ALL_SUBARCH_DIRS
      do
  #ant build first
      ANT_BUILD_FILE="$SUBARCHS_DIR/$SUBARCH_DIR/build.xml" 
      
      if [ -a $ANT_BUILD_FILE ]
	  then
	  echo $ANT_BUILD_FILE
	  ant -f $ANT_BUILD_FILE  $ANT_ARGUMENT
	  if [ $? -ne 0 ] 
	      then
	      echo Failed when running ant on $ANT_BUILD_FILE
	      echo Test just this with the command:
	      echo "    " ant -f $ANT_BUILD_FILE  $ANT_ARGUMENT

	      exit $?
	  fi
      fi

  #now cmake build
      CMAKE_LISTS_FILE="$SUBARCHS_DIR/$SUBARCH_DIR/CMakeLists.txt" 
      if [ -a $CMAKE_LISTS_FILE ]
	  then

      #echo $CMAKE_LISTS_FILE
	  BUILD_DIR="$SUBARCHS_BUILD_DIR/$SUBARCH_DIR"
	  mkdir -p $BUILD_DIR

	  pushd $BUILD_DIR > /dev/null

	  if [ -n "$CMAKE_CONFIGURE" ] 
	      then
	      $CMAKE_CMD "$SUBARCHS_DIR/$SUBARCH_DIR"
	      if [ $? -ne 0 ] 
		  then
		  echo Failed to configure with $CMAKE_CMD in "$SUBARCHS_DIR/$SUBARCH_DIR"
		  echo Test just this with the command:
		  echo "    " cd $BUILD_DIR\; $CMAKE_CMD "$SUBARCHS_DIR/$SUBARCH_DIR"
		  exit $?
	      fi
	  fi
	  make $MAKE_ARGUMENT
	  if [ $? -ne 0 ] 
	      then
	      echo Failed build in "$SUBARCHS_DIR/$SUBARCH_DIR"
	      echo Test just this with the command
	      echo "    " make -C $BUILD_DIR  $MAKE_ARGUMENT
	      exit $?
	  fi
	  popd
      fi
      
  #pushd "$SUBARCHS_DIRS$SUBARCH_DIR"  > /dev/null
#  pwd
 # popd > /dev/null
    done
fi


if [ -n "$BUILD_INSTANTS" ]
    then 

    for INSTANT_DIR in $ALL_INSTANT_DIRS
      do
  #ant build first
      ANT_BUILD_FILE="$INSTANTS_DIR/$INSTANT_DIR/build.xml" 
      
      echo $ANT_BUILD_FILE

      if [ -a $ANT_BUILD_FILE ]
	  then
	  echo $ANT_BUILD_FILE
	  ant -f $ANT_BUILD_FILE  $ANT_ARGUMENT
	  if [ $? -ne 0 ] 
	      then
	      echo Failed when running ant on $ANT_BUILD_FILE
	      echo Test just this with the command
	      echo "    " ant -f $ANT_BUILD_FILE  $ANT_ARGUMENT

	      exit $?
	  fi
      fi

  #now cmake build
      CMAKE_LISTS_FILE="$INSTANTS_DIR/$INSTANT_DIR/CMakeLists.txt" 
      if [ -a $CMAKE_LISTS_FILE ]
	  then

      #echo $CMAKE_LISTS_FILE
	  BUILD_DIR="$INSTANTS_BUILD_DIR/$INSTANT_DIR"
	  mkdir -p $BUILD_DIR

	  pushd $BUILD_DIR > /dev/null

	  if [ -n "$CMAKE_CONFIGURE" ] 
	      then
	      $CMAKE_CMD "$INSTANTS_DIR/$INSTANT_DIR"
	      if [ $? -ne 0 ] 
		  then
		  echo Failed to configure with $CMAKE_CMD in "$INSTANTS_DIR/$INSTANT_DIR"
		  echo Test just this with the command
		  echo "    " cd $BUILD_DIR\; $CMAKE_CMD "$INSTANTS_DIR/$INSTANT_DIR"
		  exit $?
	      fi
	  fi
	  make $MAKE_ARGUMENT
	  if [ $? -ne 0 ] 
	      then
	      echo Failed build in "$INSTANTS_DIR/$INSTANT_DIR"
	      echo Test just this with the command
	      echo "    " make -C $BUILD_DIR  $MAKE_ARGUMENT
	      exit $?
	  fi
	  popd
      fi
      


  #pushd "$INSTANTS_DIRS$INSTANT_DIR"  > /dev/null
#  pwd
 # popd > /dev/null
    done
fi
