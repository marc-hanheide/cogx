# Protocol Buffers as domain objects

# bit of hack to get the absolute path for the proto files
GET_FILENAME_COMPONENT(ABS_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)                
                      
PROTOBUF_GENERATE_CPP(GEN_CPP GEN_HPP
                      PROTOFILES ${ABS_SRC_DIR}/cdsr_messages.proto
                      PROTOROOT  ${ABS_SRC_DIR}
                      OUTPATH    ${CMAKE_CURRENT_BINARY_DIR})

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_BINARY_DIR})

ADD_EXECUTABLE(cdsr_test main.cc ${GEN_CPP} ${GEN_HPP})
                    
TARGET_LINK_LIBRARIES(cdsr_test rsbcore)                     
               
INSTALL(TARGETS cdsr_test RUNTIME DESTINATION bin)                                  