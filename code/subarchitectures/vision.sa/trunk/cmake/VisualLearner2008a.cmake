cmake_minimum_required (VERSION 2.8)
# Include the vision VisualLearner component
# To be included in the CMakeLists.txt of vision.sa

option(BUILD_SA_VISION_VISLEARNER "Build UOL visual learning component" NO) # default NO until fully integrated

# The core of the VisualLearner is written in Matlab and compiled into a CTF archive.
# If Matlab Compiler is available on the system, the CTF archive can be built with it.
# If the Compiler is missing, a prebuilt CTF archive has to be used.
# If the use of Matlab is not desired, a Fake Visual Learner can be built instead
# of the real one. In this case libVisualLearnerProxy_fake.so is created.
option(BUILD_SA_VISION_VISLEARNER_CTF "Build Matlab CTF for UOL visual learner; off=use prebuilt CTF" NO)
mark_as_advanced(BUILD_SA_VISION_VISLEARNER_CTF)
option(BUILD_SA_VISION_VISLEARNER_FAKE "Build a Fake Visual Learner, without Matlab dependencies" NO)

if (BUILD_SA_VISION_VISLEARNER)

   set(VISUALLEARNER_LIBRARY_DIRS)
   if (BUILD_SA_VISION_VISLEARNER_FAKE)
      set (LIB_VLPROXY_NAME "VisualLearnerProxy_fake")

      # Build the fake proxy
      add_subdirectory(src/matlab/cogxBuild/Proxy)
      set(VISUALLEARNER_LIBRARY_DIRS ${VISUALLEARNER_LIBRARY_DIRS} ${VisualLearnerProxy_BINARY_DIR})

      set(VISUALLEARNER_LIBRARIES   ${LIB_VLPROXY_NAME})
   else (BUILD_SA_VISION_VISLEARNER_FAKE)
      set (LIB_VLPROXY_NAME "VisualLearnerProxy")

      # Build the CTF & the Proxy or use prebuilt files
      if (BUILD_SA_VISION_VISLEARNER_CTF)

         # Build the CTF
         if (NOT BUILD_SA_VISION_VISLEARNER_FAKE)
            add_subdirectory(src/matlab/cogxBuild/prjdeploy)
            set(VISUALLEARNER_LIBRARY_DIRS ${VISUALLEARNER_LIBRARY_DIRS} ${VisualLearnerCtf_BINARY_DIR}/build)
         endif(NOT BUILD_SA_VISION_VISLEARNER_FAKE)

         # Build the real proxy
         add_subdirectory(src/matlab/cogxBuild/Proxy)
         set(VISUALLEARNER_LIBRARY_DIRS ${VISUALLEARNER_LIBRARY_DIRS} ${VisualLearnerProxy_BINARY_DIR})

      else (BUILD_SA_VISION_VISLEARNER_CTF)
         add_subdirectory(prebuilt/VisualLearner)
         # VISUALLEARNER_LIBRARY_DIRS set in included CMakeLists
      endif (BUILD_SA_VISION_VISLEARNER_CTF)

      set(VISUALLEARNER_LIBRARIES   ${LIB_VLPROXY_NAME} VisualLearnerCtf)
   endif (BUILD_SA_VISION_VISLEARNER_FAKE)


   add_subdirectory(src/c++/vision/components/VisualLearner)
   # Build the VisualLearner
   #if (BUILD_SA_VISION_VISLEARNER_FAKE)
   #   add_subdirectory(src/c++/vision/components/VisualLearner)
   #else()
   #   # Check if the VisualLearener can be built
   #   find_library(XXX_VISUAL_LEARNER_MATLAB_CTF
   #      VisualLearnerCtf
   #      PATHS
   #      ${VISUALLEARNER_LIBRARY_DIRS}
   #      ${OUTPUT}/lib
   #      )
   #   if (XXX_VISUAL_LEARNER_MATLAB_CTF)
   #      add_subdirectory(src/c++/vision/components/VisualLearner)
   #   else()
   #      message(
   #         "Library 'VisualLearnerCtf' not found.\n"
   #         "  If you have the Matlab Compiler, enable the option\n"
   #         "     BUILD_SA_VISION_VISLEARNER_CTF (Advanced)\n"
   #         "  If you don't have the Matlab Compiler, put prebuilt libraries into\n"
   #         "     vision.sa/config/prebuilt.\n"
   #         "  If you don't like Matlab, build the Fake Visual Learner by enabling\n"
   #         "     BUILD_SA_VISION_VISLEARNER_FAKE\n"
   #         "  Then run 'ccmake' again.\n"
   #         "  (For now, VisualLearner won't be built)\n"
   #         )
   #   endif()
   #   unset(XXX_VISUAL_LEARNER_MATLAB_CTF CACHE)
   #endif ()
endif (BUILD_SA_VISION_VISLEARNER)

