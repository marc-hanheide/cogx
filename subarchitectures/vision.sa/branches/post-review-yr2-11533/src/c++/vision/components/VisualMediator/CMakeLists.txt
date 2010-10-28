include_directories(.)
include_directories(${COGX_ROOT}/subarchitectures/binder/src/c++/autogen)
include_directories(${COGX_ROOT}/subarchitectures/binder/src/c++/)
include_directories(${COGX_ROOT}/tools/beliefmodels/src/c++/autogen)

# Set the sources.
SET(SOURCES VisualMediator.cpp)
 
add_cast_component(VisualMediator ${SOURCES})
link_cast_component(${CAST_COMPONENT_NAME} VisionData BinderAbstract BeliefModels DomainModel)

