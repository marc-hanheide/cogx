project(SequentialAdaBoost)

# Set build type
# set(CMAKE_BUILD_TYPE Release)

# Set the sources.
set(SOURCES 
      SequentialAdaBoostClassifier.cpp
      rangeExample.cpp
      rangeExampleGeneral.cpp
      example.cpp
      hypothesis.cpp
      thresholdHypothesis.cpp
      thresholdHypothesisGeneral.cpp
      strongClassifierGeneralized.cpp
      dynamictable.hxx
      features/feature2.cpp
      features/feature2_sd.cpp
      features/feature3.cpp
      features/feature5.cpp
      features/feature6.cpp
      features/feature7.cpp
      features/feature8.cpp
      features/feature8_sd.cpp
      features/feature9.cpp
      features/feature9_sd.cpp
      features/feature13.cpp
      features/feature14.cpp
      features/feature16.cpp
      features/feature17.cpp
      features/feature17_2.cpp
      features/feature18.cpp
      features/feature18_2.cpp
      features/feature19.cpp
      features/feature19_2.cpp
      features/feature20.cpp
      features/feature21.cpp
      features/feature22.cpp
      features/feature23.cpp
      features/feature25.cpp
      features/feature26.cpp
      features/feature27.cpp
      features/feature27_sd.cpp
      features/feature28.cpp
      features/feature28_sd.cpp
      features/feature29.cpp
      features/feature29_sd.cpp
      features/feature30.cpp
      features/feature31.cpp
      features/feature32.cpp
      features/feature33.cpp
      features/feature34.cpp
      features/feature_cent.cpp
      features/feature.cpp
      features/featureValues.cpp
      )

set(HEADERS
      SequentialAdaBoostClassifier.h
      rangeExample.h
      rangeExampleGeneral.h
      example.h
      dynamictable.h
      hypothesis.h
      thresholdHypothesis.h
      thresholdHypothesisGeneral.h
      strongClassifierGeneralized.h
      ada_global.h
      features/all_features.h
      features/feature2.h
      features/feature2_sd.h
      features/feature3.h
      features/feature5.h
      features/feature6.h
      features/feature7.h
      features/feature8.h
      features/feature8_sd.h
      features/feature9.h
      features/feature9_sd.h
      features/feature13.h
      features/feature14.h
      features/feature16.h
      features/feature17.h
      features/feature17_2.h
      features/feature18.h
      features/feature18_2.h
      features/feature19.h
      features/feature19_2.h
      features/feature20.h
      features/feature21.h
      features/feature22.h
      features/feature23.h
      features/feature25.h
      features/feature26.h
      features/feature27.h
      features/feature27_sd.h
      features/feature28.h
      features/feature28_sd.h
      features/feature29.h
      features/feature29_sd.h
      features/feature30.h
      features/feature31.h
      features/feature32.h
      features/feature33.h
      features/feature34.h
      features/feature_cent.h
      features/feature.h
      features/featureValues.h
      )

add_library(${PROJECT_NAME} SHARED ${SOURCES})

target_link_libraries(${PROJECT_NAME} gslcblas gsl)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

