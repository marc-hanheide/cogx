
find_package(PythonInterp) # PYTHON_EXECUTABLE

macro(add_pylib_module _name)
   set(PYLIB_MODULE ${_name})
   set(PYLIB_TARGET ${PYLIB_MODULE}.pylib)
   file(GLOB_RECURSE moduleFiles *.py)
   file(GLOB_RECURSE moduleFilesRel RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.py)
   # Copy python files to binary dir
   add_custom_command(
      OUTPUT __init__.py
      COMMAND cp --parents ${moduleFilesRel} ${CMAKE_CURRENT_BINARY_DIR}/
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      DEPENDS ${moduleFiles}
      )
   # Compile python files
   add_custom_command(
      OUTPUT __init__.pyc
      COMMAND ${PYTHON_EXECUTABLE} -c 'import compileall\;compileall.compile_dir\("."\)'
      DEPENDS ${moduleFiles}
      DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/__init__.py
      )
   # Put the compiled files in a zip archive
   # Zip from one level higher to include the module name in archive
   foreach(fn ${moduleFilesRel})
      LIST(APPEND compiledFilesRel "${PYLIB_MODULE}/${fn}c")
   endforeach(fn)
   add_custom_command(
      OUTPUT ${PYLIB_TARGET}
      COMMAND zip -ru ${PYLIB_MODULE}/${PYLIB_TARGET} ${compiledFilesRel}
      DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/__init__.pyc
      DEPENDS ${moduleFiles}
      WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/..
      )

   add_custom_target(${PYLIB_MODULE}PyLib ALL
      DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${PYLIB_TARGET}
      )
   install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PYLIB_TARGET}
      DESTINATION ${COGX_ROOT}/output/python
      )
endmacro(add_pylib_module _name)

add_subdirectory(dist-packages/ObjectRecognizer)
add_subdirectory(apps/castmodule)
