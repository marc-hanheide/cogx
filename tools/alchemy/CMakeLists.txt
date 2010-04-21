if(APPLE)
  add_custom_target(ALL alchemy
                   COMMAND make -f makefile_MAC
                   WORKING_DIRECTORY ${COGX_ROOT}/tools/alchemy/src
                   COMMENT "Building alchemy")
else()
  add_custom_target(ALL alchemy
                   COMMAND make
                   WORKING_DIRECTORY ${COGX_ROOT}/tools/alchemy/src
                   COMMENT "Building alchemy")
endif()
