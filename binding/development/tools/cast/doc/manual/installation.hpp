/**
 
   \page man_installation CAST Installation

\section dependencies Platforms and Dependencies 


CAST has been installed and run successfully on a variety of plaforms,
including Ubuntu, CentOS, Fedora and Mac OS X.  Installing CAST from
source requires the following packages. Most platforms provides these
as packages. On OS X we recommend <a
href="http://www.macports.org">Macports</a> for setting up a build
environment. The list below includes Ubuntu and OSX packages that can
be installed from the standard repositories.

 - g++ (part of GCC, tested with v 4.3.2). 
  - Ubuntu: g++
  - OSX: <a href="http://developer.apple.com/technology/xcode.html">Xcode</a>

 - <a href="http://java.sun.com">Sun's Java JDK</a>, preferably version 6. 
  - Ubuntu: sun-java6-jdk
  - OSX: already installed 

 - <a href="http://www.cmake.org">Cmake</a> version 2.6 or above. 
  - Ubuntu: cmake
  - OSX (macports): cmake

 - <a href="http://ant.apache.org">Apache Ant</a>. 
  - Ubuntu: ant
  - OSX (macports): apache-ant

 - <a href="http://www.zeroc.com/ice.html">Ice</a> version 3.3 or above for C++ and Java. 
  - Ubuntu: libzeroc-ice33-dev and libzeroc-ice-java.
  - OSX (macports): db46 +java and ice-java and ice-cpp

 - <a href="http://www.boost.org">Boost</a> version 1.35 or above. 
  - Ubuntu: libboost1.35-dev
  - OSX (macports): boost


It is important that the the Ice.jar and ant-ice.jar files installed
by the Ice java package are included in your classpath. You can add...

\verbatim
export CLASSPATH=/usr/share/java/Ice.jar:/usr/share/java/ant-ice.jar:$CLASSPATH
\endverbatim

... to your .bashrc file, or similar. If you have multiple versions of
Java installed on Ubuntu you can use

\verbatim
sudo update-alternatives --config java
\endverbatim

... to select the correct installation.

\section installation Installation

<ol>

<li> Untar the cast-*.tar.gz file. We'll call the resulting directory
$CAST_ROOT. Change into that directory.</li>


<li> Edit $CAST_ROOT/build.xml so that the "install.prefix" property
is the prefix where you'd like to install CAST on your system. The
default is /usr/local.</li>

<li> Compile the Java elements of CAST using ant:

\verbatim ant build \endverbatim

This should produce a directory output/classes under $CAST_ROOT which
contains all the Java parts of CAST.</li>

<li>Install the jar file:

\verbatim sudo ant install \endverbatim

This will copy the file cast.jar to <install.prefix>/share/java for
later use. If there are problems with this you can do it manually:

\verbatim
cp $CAST_ROOT/output/jars/cast.jar <install.prefix>/java/java
\endverbatim

</li>

<li> Configure the C++ parts of CAST using cmake. Do this by creating
the $CAST_ROOT/BUILD directory, changing into it, then running ccmake
on the directory above. ccmake is a cross-platform tool which produces
makefiles which we will later use to build CAST. The command will
produce a GUI window with a number of options. The default values
should be fine, unless you wish to change CMAKE_INSTALL_PREFIX to be a
different install prefix. This should be the same as you've used in
build.xml. To create the build files press 'c' (to configure the build
files) until you are able to press 'g' (to generate the build
files). If you encounter errors, follow the instructions in the error
messages to fix them.


\verbatim
mkdir $CAST_ROOT/BUILD
cd $CAST_ROOT/BUILD
ccmake ..
\endverbatim


<li> Once you have configured the build with cmake you can build and
install CAST. Do this by changing to $CAST_ROOT and running make
install within the build directory:

\verbatim
cd $CAST_ROOT
make -C BUILD 
sudo make -C BUILD install
\endverbatim

If this completes successfully, the file
<install.prefix>/bin/cast-server should exist. If this does exist,
then CAST is installed!

<li> Configure your system environment to find CAST. This means
including <install.prefix>/bin in your PATH, and
<install.prefix>/share/java/cast.jar in your CLASSPATH.

</ol>

\section errors Common Installation Errors

<ol>

<li>If ant exits with an error like:

\verbatim
BUILD FAILED
/tmp/cogx-code/build.xml:53: The following error occurred while executing this line:
/tmp/cogx-code/build.xml:24: The following error occurred while executing this line:
/tmp/cogx-code/tools/cast/build.xml:12: taskdef class Slice2JavaTask cannot be found
\endverbatim

you most likely forgot to set the CLASSPATH so that it include the location of ant-ice.jar (please see top of page)


<li>If you get compilation errors like:

\verbatim
compile:
    [javac] Compiling 280 source files to /tmp/cogx-code/output/classes
    [javac] /tmp/cogx-code/tools/cast/src/java/cast/CASTException.java:14: package Ice does not exist
    [javac] public class CASTException extends Ice.UserException
    [javac]                                       ^
    [javac] /tmp/cogx-code/tools/cast/src/java/cast/AlreadyExistsOnWMException.java:33: package IceInternal does not exist
    [javac]     __write(IceInternal.BasicStream __os)
    [javac]   
    ...
\endverbatim

you forgot to add the path to Ice.jar to your CLASSPATH as described
above.  </ol>


*/
