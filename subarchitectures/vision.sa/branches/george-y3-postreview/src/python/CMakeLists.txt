
if(BUILD_PYTHON_COMPONENTS)
   add_subdirectory (slicegen)
   # This is the old ObjectRecognizer that was implemented in Python
   #if(BUILD_SA_VISION_OBJRECOGNIZER)
   #   add_subdirectory (ObjectRecognizer)
   #endif(BUILD_SA_VISION_OBJRECOGNIZER)
endif(BUILD_PYTHON_COMPONENTS)
