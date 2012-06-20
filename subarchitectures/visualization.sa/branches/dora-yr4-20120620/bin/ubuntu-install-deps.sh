#!/bin/bash

# List compiled for Ubuntu 9.10
libs="libqt4-dev"  # huge list of dependencies

# OpenGL
libs="$libs  libgl1-mesa-dev  libglu1-mesa-dev"

# Font rendering in opengl
libs="$libs  libftgl-dev"

# LuaGlScript
libs="$libs  liblua5.1-0-dev  libtolua++5.1-dev"

# TomGine serialization
libs="$libs  libboost-serialization-dev  libboost-iostreams-dev"

apt-get install $libs

