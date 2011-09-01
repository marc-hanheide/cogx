SET(AK_IMAGE_EXAMPLE 0 CACHE BOOL "tests for ak::image" )
IF(AK_IMAGE_EXAMPLE )
rosbuild_add_executable(example_akimage akimagetest.cpp)
ENDIF(AK_IMAGE_EXAMPLE )