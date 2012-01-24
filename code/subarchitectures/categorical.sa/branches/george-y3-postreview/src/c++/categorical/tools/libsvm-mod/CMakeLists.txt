PROJECT(libsvm-mod)
SET(VERSION "1.3.6")

# Set build type
# SET(CMAKE_BUILD_TYPE Release)

# Add definition with version
ADD_DEFINITIONS(-DVERSION=\\"${VERSION}\\")

# Applications and libraries
SET(LIBSVMMOD_SOURCES src/allocMatrix.cpp src/localkernels.cpp src/localkernel2.cpp src/somekernels.cpp 
                      src/svm.cpp src/svreduction.cpp src/confidence.cpp src/predict.cpp src/time.cpp)
SET(LIBSVMMOD_HEADERS src/allocMatrix.h src/localkernels.h src/localkernel2.h src/somekernels.h
                      src/svm.h src/svreduction.h src/confidence.h src/predict.h src/time.h)
ADD_LIBRARY(svm-mod SHARED ${LIBSVMMOD_SOURCES})

ADD_EXECUTABLE(svm-predict ${LIBSVMMOD_SOURCES} src/svm-predict.cpp)
ADD_EXECUTABLE(svm-train ${LIBSVMMOD_SOURCES} src/svm-train.cpp)
ADD_EXECUTABLE(svm-scale src/svm-scale.cpp)

# Installing
INSTALL(TARGETS svm-mod LIBRARY DESTINATION lib)
INSTALL(TARGETS svm-predict svm-train svm-scale RUNTIME DESTINATION bin)