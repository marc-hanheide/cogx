# if CAST_FOUND not set, kick up a fuss
IF (NOT CAST_FOUND)

  message(SEND_ERROR "UseCAST has been called without CAST being found. Use FindCAST first.")

ELSE (NOT CAST_FOUND)

  set(CMAKE_MODULE_PATH ${CAST_CMAKE_DIR} ${CMAKE_MODULE_PATH})

  include(CASTDeps)

  include(UseIce)
  include(UseBoost)

  include_directories(${CAST_INCLUDE_DIR})
  link_directories(${CAST_LIBRARY_DIR})

ENDIF (NOT CAST_FOUND)




