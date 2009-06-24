include_directories(..)

SET(SOURCES AbstractTTSServer.cpp)
SET(HEADERS AbstractTTSServer.hpp)
ADD_LIBRARY(AbstractTTSServer SHARED ${SOURCES} ${HEADERS})
TARGET_LINK_LIBRARIES(AbstractTTSServer TTS)
TARGET_LINK_LIBRARIES(AbstractTTSServer CDL CASTCore CASTArchitecture )
INSTALL(TARGETS AbstractTTSServer LIBRARY DESTINATION lib)

add_cast_component(TTSServerDoubleTalk TTSServerDoubleTalk.cpp)
link_cast_component(${CAST_COMPONENT_NAME} AbstractTTSServer)

