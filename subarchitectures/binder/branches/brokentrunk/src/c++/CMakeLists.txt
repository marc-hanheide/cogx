include_directories(.)

add_library(BinderAbstract SHARED BindingWorkingMemoryReader.hpp
BindingWorkingMemoryReader.cpp BindingWorkingMemoryWriter.hpp
BindingWorkingMemoryWriter.cpp)
target_link_libraries(BinderAbstract CDL CASTCore CASTArchitecture)
target_link_libraries(BinderAbstract BinderEssentials)
install(TARGETS BinderAbstract LIBRARY DESTINATION lib)


add_cast_component(ExampleProxyPusher ExampleProxyPusher.hpp ExampleProxyPusher.cpp)
link_cast_component(${CAST_COMPONENT_NAME} BinderAbstract)

