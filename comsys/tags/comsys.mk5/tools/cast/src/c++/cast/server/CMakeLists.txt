set(sources ComponentServer.cpp ComponentCreator.cpp CASTComponentFactory.cpp CASTTimeServer.cpp)

set(headers ComponentCreator.hpp CASTComponentFactory.hpp CASTTimeServer.hpp)

add_executable (cast-server-c++ ${sources} ${headers}) 

target_link_libraries(cast-server-c++ dl)
target_link_libraries(cast-server-c++ Ice)
target_link_libraries(cast-server-c++ IceUtil)
target_link_libraries(cast-server-c++ CDL CASTCore CASTArchitecture)

install(TARGETS cast-server-c++ RUNTIME DESTINATION bin)
install(FILES ${headers} DESTINATION include/cast/server)
