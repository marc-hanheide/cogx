# Author: Charles Gretton (charles.gretton@gmail.com)
project (test_harness__CAST_RPC)

# use cmake files packaged with CAST
include(UseCAST)

# add a subdirectory containing another CMakeLists.txt file
add_subdirectory(src/c++-0x)

