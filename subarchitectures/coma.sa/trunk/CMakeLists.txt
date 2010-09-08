cmake_minimum_required (VERSION 2.6)
project (Coma)

if(DISPLAY_COMA_PB)
   include_directories(src/c++)
   add_subdirectory(src/c++)
endif(DISPLAY_COMA_PB)
