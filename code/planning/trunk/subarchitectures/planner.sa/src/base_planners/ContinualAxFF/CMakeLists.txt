add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/ff
                   COMMAND make
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_custom_target(aff ALL 
                  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/ff)