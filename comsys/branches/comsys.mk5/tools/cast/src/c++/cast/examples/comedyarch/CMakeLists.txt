include(Slice2Cpp)
cast_slice2cpp(GEN_CPP GEN_HPP ComedyEssentials.ice)
ADD_LIBRARY(ComedyShared SHARED ${GEN_CPP} ${GEN_HPP})
TARGET_LINK_LIBRARIES(ComedyShared CASTArchitecture)
INSTALL(TARGETS ComedyShared LIBRARY DESTINATION lib/cast)

add_cast_component_internal(StraightMan StraightMan.cpp StraightMan.hpp)
LINK_CAST_COMPONENT(StraightMan ComedyShared)

add_cast_component_internal(FunnyMan FunnyMan.cpp FunnyMan.hpp)
LINK_CAST_COMPONENT(FunnyMan ComedyShared)

add_cast_component_internal(AudienceMember AudienceMember.cpp AudienceMember.hpp)
LINK_CAST_COMPONENT(AudienceMember ComedyShared)

add_cast_component_internal(AssistantDirector AssistantDirector.cpp AssistantDirector.hpp)
LINK_CAST_COMPONENT(AssistantDirector ComedyShared)

add_cast_component_internal(Director Director.cpp Director.hpp)
LINK_CAST_COMPONENT(Director ComedyShared)
