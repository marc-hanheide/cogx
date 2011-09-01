V4R Project 

>>>> Usage

You have to checkout the hole structure. 
svn checkout https://svn.acin.tuwien.ac.at/v4r/trunk/ v4r-framework --username username
You must set the V4R_DIR variable in your bash, bashrc or .profile using
export V4R_DIR="$HOME/projects/v4r-framework"
You header files should be included wiht <v4r/Project/file.h>
You should not use local include paths like 
INCLUDE_DIRECTORIES(.)

If you keep that syntax you can use parts of our lib with the following CMake syntax in your project

include(v4r.cmake)

include_directories(${V4R_DIR})
LINK_DIRECTORIES(${V4R_DIR}/lib)
SET ( V4R_LIBRARIES  v4rmarker )

>>>> Debug
if you like to compile with debug flags use 
cmake -DCMAKE_BUILD_TYPE=Debug ..

>>>> Eclipse CDT4

if you like to use eclipse create a build folder next to the current like:
acin
 |-> build
 |-> src (which is the svn checkout)
step into build and execute cmake with the following options
cmake -G"Eclipse CDT4 - Unix Makefiles" -D CMAKE_BUILD_TYPE=Debug ../src


>>>> OpenCV
If you like to us a other opencv change edit the PKG_CONFIG_PATH variable --> opencv.cmake


