# Protocol Buffers as domain objects

# bit of hack to get the absolute path for the proto files
GET_FILENAME_COMPONENT(ABS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)                
                      
PROTOBUF_GENERATE_CPP(GEN_CPP GEN_HPP
                      PROTOFILES ${ABS_SRC_DIR}/cdsr_messages.proto
                      PROTOROOT  ${ABS_SRC_DIR}
                      OUTPATH    ${CMAKE_CURRENT_BINARY_DIR})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})


add_library(cdsr_messages SHARED ${GEN_CPP} ${GEN_HPP})
target_link_libraries(cdsr_messages rsbcore)
INSTALL(TARGETS cdsr_messages LIBRARY DESTINATION lib)

add_executable(cdsr_test main.cc)                    
target_link_libraries(cdsr_test cdsr_messages rsbcore)                     
install(TARGETS cdsr_test RUNTIME DESTINATION bin)                                  

option(BUILD_ROOM_COORDS "Build component to transform room coords. Requires CGAL." NO)
if(BUILD_ROOM_COORDS)
	find_package(CGAL REQUIRED)
	add_definitions(${CGAL_DEFINITIONS})        
    
	include_directories(${CGAL_INCLUDE_DIR})
	add_executable(RoomCoordinateFrameTransformer RoomCoordinateFrameTransformer.cpp)
	target_link_libraries(RoomCoordinateFrameTransformer cdsr_messages ${CGAL_LIBRARIES})                     
	install(TARGETS RoomCoordinateFrameTransformer RUNTIME DESTINATION bin)                                  
endif(BUILD_ROOM_COORDS)

