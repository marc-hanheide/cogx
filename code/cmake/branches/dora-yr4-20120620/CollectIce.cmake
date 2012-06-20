# A macro that is used to build a list of ICE files to be included
# in a single ICE interface library.
# @author: Marko Mahnič
# @created: July 2010

set(COLLECT_SLICE_INCLUDE_DIRS
   -I/usr/share/slice
   -I/usr/share/Ice/slice
   -I${CAST_SLICE_DIR}
   )
set(COLLECT_ICE_FILES "")

macro(collect_ice_files _dir)
   list(APPEND COLLECT_SLICE_INCLUDE_DIRS "-I${_dir}")

   if(${ARGC} LESS "2")
      set(_files "*.ice")
   else(${ARGC} LESS "2")
      set(_files ${ARGN})
   endif(${ARGC} LESS "2")

   foreach(_file ${_files})
      file(GLOB _ices ${_dir}/${_file}) 
      list(APPEND COLLECT_ICE_FILES ${_ices})
   endforeach(_file ${_files})
endmacro(collect_ice_files)
