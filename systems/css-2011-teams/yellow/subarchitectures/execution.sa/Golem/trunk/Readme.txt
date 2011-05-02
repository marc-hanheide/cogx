OVERVIEW
========

Golem is a C++ toolkit designed primarly for a versatile control of open-chain
robotic manipulators. It features a bunch of interfaces which enable
multi-level asynchronous control of a robotic manipulator:

- direct control of position and velocity of the manipulator joints
- control in the workspace coordinates (using inverse kinematics solver alone)
- path planning with collision detection
- real-time velocity control with path planning and collision detection

Thanks to physics simulator, all robotic experiments can be performed in a virtual
world with virtual manipulators and objects, as well as with the real ones.
At present Golem provides drivers to 5DOF Neuronics Katana 300/450 robotic arms,
as well as to a generic virtual manipulator capable of simulating a generic
open-chain robotic manipulator. The generic virtual manipulator has two presets
for a virtual 5DOF Katana 300/4500 and a simple 6DOF manipulator.

* * *

Golem as a core part of my PhD contains several other tools which enables a robot
to learn to predict consequences of its own actions. They involve various algorithms
for modelling distributions, prediction and control learning, etc. (still ongoing
research and software development).


REQUIREMENTS
============

Golem has been design to work on Windows-based and Linux-based platforms,
and it has been tested on:

- Windows XP SP3 x86
- Windows 7 x64
- Ubuntu 10.04 32 bit

Golem requires Visual C++ 2005/2008 on Windows platforms, or GNU C++ compiler
with cmake makefile tool on Linux platforms. It also requires the following
software components:

- NVIDIA (former AGEIA) PhysX physics simulator
- Freeglut OpenGL utility toolkit
- Expat XML parser
- ZeroC Ice object-oriented middleware


WINDOWS INSTALLATION
====================

1. Installation of PhysX.

Golem requires installation of PhysX SDK which can be downloaded from NVIDIA site:

	http://developer.nvidia.com/object/physx_downloads.html

PhysX SDK requires PhysX System Software which is included in NVIDIA graphics driver.
Alternatively a separate PhysX SDK System Software can be installed.

Installation is simple and straightforward. If necessary - please refer to the
PhysX installation guide.

The current version of Golem has been tested with:

	PhysX SDK Version 2.8.1 (file "PhysX_2.8.1_SDK_Core.msi")


2. Installation of ZeroC Ice middleware.

The installation is optional. ZeroC Ice can be downloaded from:

	http://www.zeroc.com/download.html

The installation version depends on the installed version of Visual Studio. ZeroC Ice
supports Visual Studio 2005 SP1, 2008 and 2010. Choose any preferable target 
installation directory further referred as to <IceDir>. The default installation
directory for Ice version 3.4.1 in Windows x64 is "C:\Program Files (x86)\ZeroC\Ice-3.4.1".


3. Installation of Freeglut and Expat.

Freeglut and Expat can be fetched from the sourceforge site:

	http://freeglut.sourceforge.net/
	http://expat.sourceforge.net/

Freeglut and Expat are distributed as compressed file archives (Expat has also
a Windows installer), so they all can be extracted in any convenient directory.
For example in "C:\Users\{user_name}\Documents\Projects", so that the full
paths to the packages are e.g.:

	C:\Users\{user_name}\Documents\Projects\freeglut-2.4.0
	C:\Users\{user_name}\Documents\Projects\expat-2.0.1


4. Installation of Golem.

Similarly to Freeglut and Expat, Golem are distributed as compressed archives.
The full paths to the packages are:

	C:\Users\{user_name}\Documents\Projects\Golem


5. Visual Studio configuration

In order to be able to compile and link Golem applications one has to set up
paths to executable files, header files and libraries of the above-mentioned 
software packages. 

In Visual Studio 2005 and 2008 this can be achieved by editing directory paths
in options:

Tools menu -> Options... -> Projects and Solutions -> VC++ Directories

The additional executable file directories are:

	<IceDir>\bin

The additional include file directories are:

	$(ProgramFiles)\NVIDIA Corporation\NVIDIA PhysX SDK\v2.8.1\SDKs\Foundation\include
	$(ProgramFiles)\NVIDIA Corporation\NVIDIA PhysX SDK\v2.8.1\SDKs\Physics\include
	$(ProgramFiles)\NVIDIA Corporation\NVIDIA PhysX SDK\v2.8.1\SDKs\Cooking\include
	$(ProgramFiles)\NVIDIA Corporation\NVIDIA PhysX SDK\v2.8.1\SDKs\PhysXLoader\include
	<IceDir>\include
	$(HomePath)\Documents\Projects\freeglut-2.4.0\include
	$(HomePath)\Documents\Projects\expat-2.0.1\lib
	$(HomePath)\Documents\Projects\Golem\include

The additional library file directories are:

	$(ProgramFiles)\NVIDIA Corporation\NVIDIA PhysX SDK\v2.8.1\SDKs\lib\Win32
	<IceDir>\lib
	$(HomePath)\Documents\Projects\freeglut-2.4.0\Release
	$(HomePath)\Documents\Projects\expat-2.0.1\win32\bin\Release
	$(HomePath)\Documents\Projects\Golem\lib\win32

The above directories can vary depending on the software version and built target.

In Visual Studio 2010 the corresponding executable file directories can be set in
PATH environment variable, for example running:

	rundll32 sysdm.cpl,EditEnvironmentVariables

Similarily the include file directories in can be set is INCLUDE environment variable
and the library file directories in LIB environment variable.


6. Building and running Golem applications

Prior to building Golem applications, Freeglut and Expat have to be built. 
Golem's VC++ projects are by default configured to work with release versions of
libraries of Freeglut and Expat (dll libraries), and they can be easily
reconfigured for other build targets.
When linking with dynamic libraries and running applications, one must also make
sure that all necessary dll library files can be found by Golem applications.



LINUX INSTALLATION
==================

There are loads of distributions of Linux around. This example shows how to
install Golem on Ubuntu 10.04.


1. Installation of GNU C++ compiler and cmake

Make sure that GNU C++ compiler and cmake are already installed on the system -
if not, they can be installed using Synaptic Package Manager. Packages names are:

	g++
	cmake
	cmake-curses-gui
	g++-multilib (on amd64 architecture only)


2. Installation of PhysX.

PhysX SDK can be downloaded from NVIDIA site (Debian installation package):

http://developer.nvidia.com/object/physx_downloads.html

The current version of Golem has been tested with PhysX SDK Version 2.8.1
which consists of 6 Debian packages, all compressed in
"PhysX_2.8.1_SDK_CoreLinux_deb.tar.gz". They have to be installed in a
following order:

	libphysx-common_2.8.1-4_i386.deb
	libphysx-2.8.1_4_i386.deb
	libphysx-dev-2.8.1_4_i386.deb
	libphysx-extras-2.8.1_4_i386.deb

and optionally:

	libphysx-samples-2.8.1_4_i386.deb
	libphysx-doc-2.8.1_4_i386.deb


3. Installation of ZeroC Ice middleware.

ZeroC Ice can be installed through Synaptic Package Manager. The following
packages have to be installed (with all other dependent packages):

	zeroc-ice33


4. Installation of Freeglut and Expat.

Freeglut and Expat can be installed through Synaptic Package Manager. The following
packages have to be installed (with all other dependent packages):

	freeglut3-dev (development files)
	libexpat1-dev (development kit)


5. Installation of Golem.

- extract the Golem file archive in any convenient directory - for example in
	
	/home/{user_name}/Documents/Projects

- create a temporary build directory e.g. "BUILD" in the cmake directory:

	mkdir /home/{user_name}/Documents/Projects/Golem/compiler/cmake/default/BUILD

- enter the build directory directory and run cmake in wizard mode:
	
	cd /home/{user_name}/Documents/Projects/Golem/compiler/cmake/default/BUILD
	ccmake -i ..

- press [c] to configure cmake and then edit cmake variables if necessary:
	
	CMAKE_INSTALL_PREFIX - the default installation directory for libraries, header files
		and Golem demonstration programs.
	LIBRARY_OUTPUT_PATH - the default output directory for building libraries
	EXECUTABLE_OUTPUT_PATH - the default output directory for building executables
	PHYSX_INCLUDE - the default path prefix for PhysX include
	PHYSX_LIBRARY - the default path prefix for PhysX library
	ICE_BIN - the default path prefix for ZeroC Ice binaries (executable files)
	ICE_INCLUDE - the default path prefix for ZeroC Ice include
	ICE_LIBRARY - the default path prefix for ZeroC Ice library

- press again [c] and then [g] to generate makefiles and exit cmake

- run make and make install to build and install Golem

	make
	make install
	
  To install files administrator priviliges may be required, so:
  
	sudo make install
	

6. Running Golem applications

Make sure that all the required libraries can be localised by demonstration programs.
Update LD_LIBRARY_PATH so that it points to the PhysX and Golem libraries.
For the bash shell in the configuration file (/home/{user_name}/.bashrc) add a line:

	export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib:/usr/local/lib:/usr/lib/PhysX/v2.8.1

then type

	source /home/{user_name}/.bashrc



FIRMWARE UPGRADE
================

Older versions of Katana 300 may not have a firmware which enables
trajectory-based control by Golem controller.

The required firmware can be found in the resource subdirectory. All Katana
axis controllers require the same firmware image (version 2.1):

	normalmotor-v2.1.hex

Firmware can be uploaded using Firmware Loader from Neuronics.
For details please refer to the Firmware Loader documentation.



AUTHOR
======

Marek Kopicki <m.s.kopicki@cs.bham.ac.uk>
School of Computer Science, University of Birmingham, Edgbaston, Birmingham, B15 2TT, UK
