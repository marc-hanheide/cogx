
set(cast_cmake_files CASTBuild.cmake CASTDeps.cmake FindIce.cmake
Slice2Cpp.cmake UseBoost.cmake UseCAST.cmake UseIce.cmake FindCAST.cmake)

set(cast_template_files ComponentTemplateCMakeLists.txt
ProjectTemplateCMakeLists.txt TopLevelTemplateCMakeLists.txt
SliceLibraryTemplateCMakeLists.txt build-template.xml
build-toplevel.xml slice2java-template.xml)


install(FILES ${cast_cmake_files} DESTINATION share/cast/cmake)
install(FILES ${cast_template_files} DESTINATION share/cast/templates)
