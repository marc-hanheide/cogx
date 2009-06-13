PROJECT(BindingSubarch)
LINK_LIBRARIES(boost_regex)


include_directories(../autogen)
include_directories(${COGX_ROOT}/tools/cast/src/c++/cast/slice)
#include_directories(.)


SET(SOURCES utils/BindingScoreUtils.cpp
	    utils/LocalClasses.cpp
	    utils/BindingUtils.cpp
	    utils/DotUtils.cpp
	    utils/Predicates.cpp
	    abstr/AbstractMonitor.cpp
	    abstr/AbstractBinder.cpp
	    abstr/AbstractBindingWMRepresenter.cpp
	    #nicks old abstr/AbstractFeatureGenerator.cpp
	    abstr/AbstractFeatureComparator.cpp
	    abstr/AbstractActiveBindingWMReader.cpp
	    ontology/BindingFeatureOntology.cpp
	    feature-utils/AbstractFeature.cpp
	    feature-utils/Features.cpp
	    feature-utils/AbstractInternalComparator.cpp
	    feature-utils/FeaturePropertiesUtils.cpp
    	    feature-utils/FeatureLoader.cpp
    	    feature-utils/FeatureHelper.cpp
    	    feature-specialization/FeatureRegistrant.cpp
	    #feature-specialization/helpers/UGLYFeatureProperties.cpp
	    feature-specialization/helpers/SalienceHelper.cpp
	    feature-specialization/helpers/ConceptHelper.cpp
	    feature-specialization/helpers/GroupHelper.cpp
	    feature-specialization/helpers/SingularHelper.cpp
	    feature-specialization/helpers/SourceIDHelper.cpp
	    feature-specialization/helpers/OtherSourceIDHelper.cpp
	    feature-specialization/helpers/ColourHelper.cpp
	    feature-specialization/helpers/ShapeHelper.cpp
	    feature-specialization/helpers/SizeHelper.cpp
	    feature-specialization/helpers/ThisProxyIDHelper.cpp
	    feature-specialization/helpers/ExistingProxyIDHelper.cpp
	    feature-specialization/helpers/SourceDataHelper.cpp
	    feature-specialization/helpers/CreationTimeHelper.cpp
	    feature-specialization/helpers/NameHelper.cpp
	    feature-specialization/helpers/RelationLabelHelper.cpp
	    feature-specialization/helpers/DebugStringHelper.cpp
	    feature-specialization/helpers/LocationHelper.cpp
	    feature-specialization/helpers/AreaIDHelper.cpp
	    feature-specialization/helpers/AspectualStateHelper.cpp
	    feature-specialization/helpers/PersonIDHelper.cpp
	    #feature-specialization/helpers/VisionHelpers.cpp
	    feature-specialization/helpers/TemporalFrameHelper.cpp)

IF (BINDING_BUILD_EXPERIMENTAL_FEATURES)
  ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES}
            feature-specialization/helpers/CoordinateHelper.cpp
            feature-specialization/helpers/LineOfSightHelper.cpp)
ELSE()
  ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCES})
ENDIF(BINDING_BUILD_EXPERIMENTAL_FEATURES)

# Add libraries to link against.
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OMNIORB4_LINK_FLAGS} ${OMNIORB4_CFLAGS})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} FrameworkInterface FrameworkIce FrameworkCore)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} CASTCore CASTArchitecture CDL)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} BindingIce)
INSTALL(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)


SET(HEADERS BindingException.hpp)
INSTALL(FILES ${HEADERS} DESTINATION include/binding/)

SET(HEADERS utils/LocalClasses.hpp
	    utils/LocalClassesSets.hpp
	    utils/BindingUtils.hpp
	    utils/BindingScoreUtils.hpp
	    utils/TypeMap.hpp
	    utils/DotUtils.hpp
	    utils/QueryUtils.hpp
	    utils/non_repeating_queue.hpp
	    utils/GraphLoader.hpp
	    utils/BasicExtractors.hpp
	    utils/Extractors.hpp
	    utils/BasicPredicates.hpp
	    utils/Predicates.hpp
	    utils/Misc.hpp)
INSTALL(FILES ${HEADERS} DESTINATION include/binding/utils/)

SET(HEADERS abstr/AbstractMonitor.hpp
	    abstr/AbstractBinder.hpp
	    abstr/AbstractBindingWMRepresenter.hpp
	    abstr/AbstractFeatureComparator.hpp
	    #nicks old abstr/AbstractFeatureGenerator.hpp
	    abstr/AbstractActiveBindingWMReader.hpp)
INSTALL(FILES ${HEADERS} DESTINATION include/binding/abstr/)

SET(HEADERS ontology/BindingFeatureOntology.hpp)
INSTALL(FILES ${HEADERS} DESTINATION include/binding/ontology/)
	
SET(HEADERS feature-utils/Features.hpp
	    feature-utils/FeatureLoader.hpp
    	    feature-utils/FeatureHelper.hpp
	    feature-utils/FeatureExtractor.hpp
	    feature-utils/FeaturePropertiesUtils.hpp
	    feature-utils/AbstractInternalComparator.hpp
	    feature-utils/AbstractFeature.hpp)
INSTALL(FILES ${HEADERS} DESTINATION include/binding/feature-utils/)

SET(HEADERS feature-specialization/FeatureRegistrant.hpp)
INSTALL(FILES ${HEADERS} DESTINATION include/binding/feature-specialization/)

SET(HEADERS feature-specialization/helpers/SalienceHelper.hpp
	    feature-specialization/helpers/ConceptHelper.hpp
	    feature-specialization/helpers/GroupHelper.hpp
	    feature-specialization/helpers/SingularHelper.hpp
	    feature-specialization/helpers/SourceIDHelper.hpp
	    feature-specialization/helpers/OtherSourceIDHelper.hpp
	    feature-specialization/helpers/ColourHelper.hpp
	    feature-specialization/helpers/ShapeHelper.hpp
	    feature-specialization/helpers/SizeHelper.hpp
	    feature-specialization/helpers/ThisProxyIDHelper.hpp
	    feature-specialization/helpers/ExistingProxyIDHelper.hpp
	    feature-specialization/helpers/SourceDataHelper.hpp
	    feature-specialization/helpers/CreationTimeHelper.hpp
	    feature-specialization/helpers/NameHelper.hpp
	    feature-specialization/helpers/RelationLabelHelper.hpp
	    feature-specialization/helpers/DebugStringHelper.hpp
	    feature-specialization/helpers/LocationHelper.hpp	    
	    feature-specialization/helpers/AreaIDHelper.hpp
	    feature-specialization/helpers/AspectualStateHelper.hpp
	    feature-specialization/helpers/PersonIDHelper.hpp
	    #feature-specialization/helpers/VisionHelpers.hpp
	    feature-specialization/helpers/TemporalFrameHelper.hpp)

IF (BINDING_BUILD_EXPERIMENTAL_FEATURES)
  INSTALL(FILES ${HEADERS} 
	    feature-specialization/helpers/ExperimentalUtil.hpp
	    feature-specialization/helpers/CoordinateHelper.hpp  
	    feature-specialization/helpers/LineOfSightHelper.hpp
	    DESTINATION include/binding/feature-specialization/helpers/)	 
ELSE()
  INSTALL(FILES ${HEADERS} DESTINATION include/binding/feature-specialization/helpers/)	    
ENDIF(BINDING_BUILD_EXPERIMENTAL_FEATURES)



