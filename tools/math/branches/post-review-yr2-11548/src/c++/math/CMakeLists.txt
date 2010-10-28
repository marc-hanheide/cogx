include_directories(.)

ADD_LIBRARY(cogxmath SHARED cogxmath_rand.cc)

INSTALL(TARGETS cogxmath LIBRARY DESTINATION lib)

