# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.6

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/plison/svn.cogx/binding/development

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/plison/svn.cogx/binding/development/BUILD

# Include any dependencies generated for this target.
include tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/depend.make

# Include the progress variables for this target.
include tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/progress.make

# Include the compile flags for this target's objects.
include tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/flags.make

tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.o: tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/flags.make
tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.o: ../tools/cast/src/c++/cast/architecture/AlwaysPositiveTaskManager.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/plison/svn.cogx/binding/development/BUILD/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.o"
	cd /home/plison/svn.cogx/binding/development/BUILD/tools/cast/src/c++/cast/architecture && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.o -c /home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/architecture/AlwaysPositiveTaskManager.cpp

tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.i"
	cd /home/plison/svn.cogx/binding/development/BUILD/tools/cast/src/c++/cast/architecture && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/architecture/AlwaysPositiveTaskManager.cpp > CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.i

tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.s"
	cd /home/plison/svn.cogx/binding/development/BUILD/tools/cast/src/c++/cast/architecture && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/architecture/AlwaysPositiveTaskManager.cpp -o CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.s

tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.o.requires:
.PHONY : tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.o.requires

tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.o.provides: tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.o.requires
	$(MAKE) -f tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/build.make tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.o.provides.build
.PHONY : tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.o.provides

tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.o.provides.build: tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.o
.PHONY : tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.o.provides.build

# Object files for target AlwaysPositiveTaskManager
AlwaysPositiveTaskManager_OBJECTS = \
"CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.o"

# External object files for target AlwaysPositiveTaskManager
AlwaysPositiveTaskManager_EXTERNAL_OBJECTS =

tools/cast/src/c++/cast/architecture/libAlwaysPositiveTaskManager.so: tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.o
tools/cast/src/c++/cast/architecture/libAlwaysPositiveTaskManager.so: tools/cast/src/c++/cast/slice/libCDL.so
tools/cast/src/c++/cast/architecture/libAlwaysPositiveTaskManager.so: tools/cast/src/c++/cast/core/libCASTCore.so
tools/cast/src/c++/cast/architecture/libAlwaysPositiveTaskManager.so: tools/cast/src/c++/cast/architecture/libCASTArchitecture.so
tools/cast/src/c++/cast/architecture/libAlwaysPositiveTaskManager.so: tools/cast/src/c++/cast/core/libCASTCore.so
tools/cast/src/c++/cast/architecture/libAlwaysPositiveTaskManager.so: tools/cast/src/c++/cast/slice/libCDL.so
tools/cast/src/c++/cast/architecture/libAlwaysPositiveTaskManager.so: tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/build.make
tools/cast/src/c++/cast/architecture/libAlwaysPositiveTaskManager.so: tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library libAlwaysPositiveTaskManager.so"
	cd /home/plison/svn.cogx/binding/development/BUILD/tools/cast/src/c++/cast/architecture && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/AlwaysPositiveTaskManager.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/build: tools/cast/src/c++/cast/architecture/libAlwaysPositiveTaskManager.so
.PHONY : tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/build

tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/requires: tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/AlwaysPositiveTaskManager.cpp.o.requires
.PHONY : tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/requires

tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/clean:
	cd /home/plison/svn.cogx/binding/development/BUILD/tools/cast/src/c++/cast/architecture && $(CMAKE_COMMAND) -P CMakeFiles/AlwaysPositiveTaskManager.dir/cmake_clean.cmake
.PHONY : tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/clean

tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/depend:
	cd /home/plison/svn.cogx/binding/development/BUILD && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/plison/svn.cogx/binding/development /home/plison/svn.cogx/binding/development/tools/cast/src/c++/cast/architecture /home/plison/svn.cogx/binding/development/BUILD /home/plison/svn.cogx/binding/development/BUILD/tools/cast/src/c++/cast/architecture /home/plison/svn.cogx/binding/development/BUILD/tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tools/cast/src/c++/cast/architecture/CMakeFiles/AlwaysPositiveTaskManager.dir/depend

