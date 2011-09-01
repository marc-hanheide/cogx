
add_executable(eucsbademo eucsbademo.c imgproj.c readparams.c eucsbademo.h readparams.h)
target_link_libraries(eucsbademo sba)

add_executable(BundleCheckerboard BundleCheckerboard.cpp)
target_link_libraries(BundleCheckerboard sba)

