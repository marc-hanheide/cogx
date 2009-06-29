include(${CAST_ROOT}/cmake/UseBoost.cmake)

set(sources WorkingMemoryAttachedComponent.cpp
WorkingMemoryWriterComponent.cpp WorkingMemoryReaderComponent.cpp
ManagedComponent.cpp SubarchitectureTaskManager.cpp WorkingMemoryChangeFilterComparator.cpp)


set(headers WorkingMemoryAttachedComponent.hpp
WorkingMemoryWriterComponent.hpp WorkingMemoryReaderComponent.hpp
ManagedComponent.hpp SubarchitectureTaskManager.hpp
ChangeFilterFactory.hpp UnmanagedComponent.hpp
WorkingMemoryChangeFilterMap.hpp
WorkingMemoryChangeFilterComparator.hpp
WorkingMemoryChangeReceiver.hpp)
 
add_library(CASTArchitecture SHARED ${sources} ${headers})

target_link_libraries(CASTArchitecture CDL)
target_link_libraries(CASTArchitecture CASTCore)

install(TARGETS CASTArchitecture LIBRARY DESTINATION lib/cast ARCHIVE DESTINATION lib/cast)
install(FILES ${headers} DESTINATION include/cast/architecture)

add_cast_component_internal(SubarchitectureWorkingMemory SubarchitectureWorkingMemory.cpp SubarchitectureWorkingMemory.hpp)
install(FILES SubarchitectureWorkingMemory.hpp DESTINATION include/cast/architecture)

add_cast_component_internal(AlwaysPositiveTaskManager AlwaysPositiveTaskManager.cpp AlwaysPositiveTaskManager.hpp)
install(FILES AlwaysPositiveTaskManager.hpp DESTINATION include/cast/architecture)

