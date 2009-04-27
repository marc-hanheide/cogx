
MACRO (add_cast_component _name)
  add_library(${_name} SHARED ${ARGN})
  target_link_libraries(${_name} CDL CASTCore CASTArchitecture)
  install(TARGETS ${_name} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
  set(CAST_COMPONENT_NAME ${_name})
ENDMACRO (add_cast_component _name)

MACRO (add_cast_component_internal _name)
  add_library(${_name} SHARED ${ARGN})
  target_link_libraries(${_name} CDL CASTCore CASTArchitecture)
  install(TARGETS ${_name} LIBRARY DESTINATION lib/cast ARCHIVE DESTINATION lib/cast)
  set(CAST_COMPONENT_NAME ${_name})
ENDMACRO (add_cast_component_internal _name)

MACRO (link_cast_component _name)
      target_link_libraries(${_name} ${ARGN})
ENDMACRO (link_cast_component _name)

MACRO (add_and_include_subdirectory _name)
      include_directories(${_name})
      add_subdirectory(${_name})
ENDMACRO (add_and_include_subdirectory _name)



