set(sources CASTUtils.cpp CASTComponent.cpp
 SubarchitectureComponent.cpp CASTComponentPermissionsMap.cpp
 CASTWorkingMemory.cpp CASTWMPermissionsMap.cpp CASTTimer.cpp)

set(headers CASTUtils.hpp CASTComponent.hpp
 SubarchitectureComponent.hpp CASTComponentPermissionsMap.hpp
 CASTWorkingMemory.hpp CASTWMPermissionsMap.hpp CASTData.hpp
 CASTWorkingMemoryInterface.hpp StringMap.hpp CASTTimer.hpp)


add_library(CASTCore SHARED ${sources} ${headers})

target_link_libraries(CASTCore CDL)

install(TARGETS CASTCore LIBRARY DESTINATION lib/cast ARCHIVE DESTINATION lib/cast)
install(FILES ${headers} DESTINATION include/cast/core)
